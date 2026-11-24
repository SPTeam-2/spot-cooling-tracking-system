"""
[파일(내부 구현 코드) 목적]
A4988 스테퍼 모터에 STEP·DIR·EN 신호를 출력하는 HW 계층
[코드 내용]
StepperHardware: BCM 핀·중복 검사, 방향 설정, 드라이버 활성화, 종료 정리
STEP 출력 두 가지: start_pulses() = PWM + 콜백 펄스 카운트(방식 A, 기본), step_once() = 한 펄스 직접 출력(방식 B, 예비)
리미트·STOP 판단은 상위 계층 담당 / 기본 연결은 pigpio 라이브러리 기반 로컬 pigpiod 연결
[플로우 간단 정리]
객체 생성 → 핀 검증·출력 초기화 → 방향 설정·EN 활성화
→ start_pulses(총 스텝, 주파수 계획) → 콜백이 상승 에지를 세며 계획대로 주파수 전환 → 총 스텝에서 PWM 정지
→ pulse_status()로 진행 확인 / stop_pulses()로 중간 정지 → finish_pulses()로 늦은 알림까지 집계 → cleanup()

방식 A 근거: hw/py/stp_pwm_count_check.py 현장 점검 (1600스텝, 초과 0, 정지 지연 1.1~1.8ms, 3회)
콜백 카운트는 Pi가 낸 펄스 수이며 모터가 실제로 간 스텝(탈조)은 알 수 없음
RESET/SLEEP 활성화와 MS 핀 실제 배선 필요 (현재 1/8: MS1·MS2 HIGH, MS3 미연결)
통신 실패 시 모의 장치로 자동 전환하지 않음 / 주입받은 공유 연결은 종료하지 않음
"""
from __future__ import annotations

import math
import threading
import time


class PulseStatus:
    """pulse_status() 결과 스냅샷. count = 콜백이 센 상승 에지 수"""
    __slots__ = ('count', 'reached', 'stopped', 'error')

    def __init__(self, count, reached, stopped, error):
        self.count, self.reached, self.stopped, self.error = count, reached, stopped, error


class _PulseRun:
    """한 번의 start_pulses() 실행 상태. 콜백 스레드와 호출 스레드가 lock으로 공유"""

    def __init__(self, total, plan):
        self.total, self.plan = total, plan
        self.next_idx = 1          # plan[0]은 시작 시 적용
        self.count = 0
        self.reached = False       # 총 스텝 도달로 정지
        self.stopped = False       # 정지 명령 완료 (도달·중간 정지·오류 공통)
        self.error = None
        self.callback = None
        self.lock = threading.Lock()


class StepperHardware:
    """STEP·DIR·EN 출력 담당. 주입받은 공유 pigpio 연결은 종료하지 않는다."""
    def __init__(self, step_pin, dir_pin, enable_pin, *, positive_dir_level,
                 reserved_pins=(5, 6), pulse_high_s=0.00001,
                 pulse_low_s=0.00001, direction_delay_s=0.00001,
                 pi=None, sleep=time.sleep, command_timeout_s=0.25, settle_s=0.05):
        pins = (step_pin, dir_pin) + (() if enable_pin is None else (enable_pin,))
        if any(type(p) is not int or not 0 <= p <= 27 for p in pins):
            raise ValueError('BCM pins must be integers in 0..27')
        if len(set(pins)) != len(pins) or set(pins).intersection(reserved_pins):
            raise ValueError('duplicate or reserved pin')
        if type(positive_dir_level) is not int or positive_dir_level not in (0, 1):
            raise ValueError('positive_dir_level must be 0 or 1')
        for value, minimum in ((pulse_high_s, 1e-6), (pulse_low_s, 1e-6), (direction_delay_s, 2e-7)):
            if type(value) not in (int, float) or not math.isfinite(value) or value < minimum:
                raise ValueError('invalid pulse or direction timing')
        if type(settle_s) not in (int, float) or not math.isfinite(settle_s) or settle_s < 0:
            raise ValueError('settle_s must be finite and non-negative')
        self.step_pin, self.dir_pin, self.enable_pin = step_pin, dir_pin, enable_pin
        self._settle = settle_s
        self._run = None
        self._positive = positive_dir_level
        self._high, self._low, self._delay = pulse_high_s, pulse_low_s, direction_delay_s
        self._sleep = sleep
        self._owned = pi is None
        self._closed = False
        self._ready = False
        self._enabled = False
        self._direction = None
        self._configured = set()
        if pi is None:
            from .pigpio_motor_connection import PigpioMotorConnection
            pi = PigpioMotorConnection(timeout_s=command_timeout_s)
        self._pi = pi
        try:
            self.check_connection()
            # EN을 먼저 비활성화한다. pigpio.write는 해당 핀을 출력 모드로도 전환한다.
            if enable_pin is not None:
                self._configure(enable_pin, 1)
            self._configure(step_pin, 0)
            self._configure(dir_pin, positive_dir_level)
            self._ready = True
        except Exception:
            try:
                self.cleanup()
            except Exception:
                pass
            raise

    def _call(self, name, *args):
        value = getattr(self._pi, name)(*args)
        if type(value) is not int or value != 0:
            raise OSError(f'pigpio {name} failed: {value!r}')

    def _configure(self, pin, level):
        # 실패한 명령도 데몬에 전달됐을 수 있으므로 해당 핀을 정리 대상에 먼저 등록한다.
        self._configured.add(pin)
        self._call('write', pin, level)
        self._call('set_mode', pin, 1)  # 출력 모드(pigpio.OUTPUT)

    def check_connection(self):
        if self._closed or not self._pi.connected:
            raise ConnectionError('pigpiod disconnected or hardware closed')

    def set_direction(self, direction):
        """논리 방향 -1/+1을 DIR 레벨로 변환하고 신호 안정 시간을 확보한다."""
        self.check_connection()
        if self._run is not None:
            raise RuntimeError('cannot change direction while pulses are running')
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('direction must be -1 or +1')
        self._sleep(self._delay)  # 이전 STEP 이후 DIR을 바꾸기 전까지 필요한 유지 시간 확보
        self._call('write', self.dir_pin, self._positive if direction == 1 else 1-self._positive)
        self._sleep(self._delay)
        self._direction = direction

    def enable(self):
        """EN을 LOW로 내려 드라이버를 활성화한다. EN 미연결 시 배선 상태를 전제로 한다."""
        self.check_connection()
        if not self._ready:
            raise RuntimeError('initialization incomplete')
        if self.enable_pin is not None:
            self._call('write', self.enable_pin, 0)
        self._sleep(self._delay)
        self._enabled = True

    # ---- 방식 B: 한 펄스 직접 출력 (예비, 기존 모의 테스트의 가짜 모터와 같은 방식) ----
    def step_once(self):
        """STEP 한 펄스를 출력한다. 출력 도중 실패해도 LOW 복귀를 시도한다."""
        self.check_connection()
        if not self._enabled or self._direction is None:
            raise RuntimeError('set direction and enable before STEP')
        if self._run is not None:
            raise RuntimeError('PWM pulses are running')
        try:
            self._call('write', self.step_pin, 1)
            self._sleep(self._high)
        finally:
            self._call('write', self.step_pin, 0)
        self._sleep(self._low)

    # ---- 방식 A: PWM + 콜백 펄스 카운트 ----
    def start_pulses(self, total, plan):
        """total개 펄스 출력 시작. plan = [(이 카운트부터, 주파수Hz), ...], 첫 항목은 (0, 시작 주파수)

        주파수 전환과 total 도달 시 정지는 콜백 안에서 카운트 기준으로 처리한다.
        """
        self.check_connection()
        if not self._enabled or self._direction is None:
            raise RuntimeError('set direction and enable before STEP')
        if self._run is not None:
            raise RuntimeError('pulses already running; finish_pulses() first')
        if type(total) is not int or total <= 0:
            raise ValueError('total must be a positive integer')
        plan = [tuple(item) for item in plan]
        if not plan or plan[0][0] != 0:
            raise ValueError('plan must start at count 0')
        last = -1
        for at, hz in plan:
            if type(at) is not int or type(hz) is not int or hz <= 0 or at <= last or at >= total:
                raise ValueError('plan counts must increase within total and Hz must be positive integers')
            last = at
        run = _PulseRun(total, plan)
        self._run = run
        try:
            run.callback = self._pi.rising_edge_callback(self.step_pin, self._on_edge)
            self._call('set_pwm', self.step_pin, plan[0][1])
        except Exception:
            with run.lock:
                run.stopped = True
            try:
                self._call('pwm_off', self.step_pin)
            except Exception:
                pass
            self._cancel_callback(run)
            self._run = None
            raise

    def _on_edge(self, gpio, level, tick):
        """콜백 스레드: 상승 에지 카운트 → 계획대로 주파수 전환 → total에서 정지"""
        run = self._run
        if run is None or level != 1:
            return
        with run.lock:
            run.count += 1
            if run.stopped:
                return                      # 정지 명령 뒤 늦게 도착한 알림도 카운트에는 포함
            try:
                if run.count >= run.total:
                    run.stopped = run.reached = True
                    self._call('pwm_off', self.step_pin)
                    return
                while run.next_idx < len(run.plan) and run.count >= run.plan[run.next_idx][0]:
                    self._call('set_pwm', self.step_pin, run.plan[run.next_idx][1])
                    run.next_idx += 1
            except Exception as exc:
                run.error = exc
                run.stopped = True
                try:
                    self._call('pwm_off', self.step_pin)
                except Exception:
                    pass

    def pulse_status(self):
        """현재 실행의 카운트·도달·정지·콜백 오류 스냅샷"""
        run = self._run
        if run is None:
            raise RuntimeError('no pulses running')
        with run.lock:
            return PulseStatus(run.count, run.reached, run.stopped, run.error)

    def stop_pulses(self):
        """PWM 즉시 정지 (duty 0). 이미 정지했으면 아무것도 하지 않는다."""
        run = self._run
        if run is None:
            return
        with run.lock:
            if run.stopped:
                return
            run.stopped = True
        self._call('pwm_off', self.step_pin)

    def finish_pulses(self):
        """정지 확인 후 settle_s 동안 늦은 알림을 기다리고 콜백을 해제한다. 최종 카운트 반환"""
        run = self._run
        if run is None:
            raise RuntimeError('no pulses running')
        self.stop_pulses()
        self._sleep(self._settle)
        self._cancel_callback(run)
        self._run = None
        with run.lock:
            return run.count

    @staticmethod
    def _cancel_callback(run):
        if run.callback is not None:
            try:
                run.callback.cancel()
            finally:
                run.callback = None

    def disable(self):
        """STEP LOW와 EN HIGH를 각각 시도한다. 한 출력이 실패해도 다른 출력을 시도한다.

        STEP LOW 쓰기는 PWM도 함께 끈다 (pigpiod write 동작). 실행 중인 콜백은 해제한다.
        """
        self._enabled = False
        run, self._run = self._run, None
        if run is not None:
            with run.lock:
                run.stopped = True
            try:
                self._cancel_callback(run)
            except Exception:
                pass
        errors = []
        for pin, level in ((self.step_pin, 0), (self.enable_pin, 1)):
            if pin is not None and pin in self._configured:
                try:
                    self._call('write', pin, level)
                except Exception as exc:
                    errors.append(exc)
        if errors:
            raise OSError('could not establish safe output levels') from errors[0]

    def cleanup(self):
        """안전 출력을 시도하고 직접 생성한 연결만 종료한다. 반복 호출을 허용한다."""
        if self._closed:
            return
        try:
            self.disable()
        finally:
            self._closed = True
            if self._owned:
                self._pi.stop()
