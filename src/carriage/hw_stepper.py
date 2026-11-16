"""
[파일(내부 구현 코드) 목적]
A4988 스테퍼 모터에 STEP·DIR·EN 신호를 출력하는 HW 계층
[코드 내용]
StepperHardware: BCM 핀·중복 검사, 방향 설정, 드라이버 활성화, 단일 STEP 출력, 종료 정리
리미트·STOP 판단은 상위 계층 담당 / 기본 연결은 대기 제한이 있는 로컬 pigpiod 연결 사용
[플로우 간단 정리]
객체 생성 → 핀 검증·출력 초기화 → 방향 설정·EN 활성화 → step_once() 반복 → cleanup()

RESET/SLEEP 활성화와 MS 핀 실제 배선 필요
STEP HIGH/LOW 최소 각각 1us, DIR setup/hold 최소 각각 200ns / Python 대기는 길어질 수 있음
통신 실패 시 모의 장치로 자동 전환하지 않음 / 주입받은 공유 연결은 종료하지 않음
"""
import math
import time


class StepperHardware:
    """STEP·DIR·EN 출력 담당. 주입받은 공유 pigpio 연결은 종료하지 않는다."""
    def __init__(self, step_pin, dir_pin, enable_pin, *, positive_dir_level,
                 reserved_pins=(5, 6), pulse_high_s=0.00001,
                 pulse_low_s=0.00001, direction_delay_s=0.00001,
                 pi=None, sleep=time.sleep, command_timeout_s=0.25):
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
        self.step_pin, self.dir_pin, self.enable_pin = step_pin, dir_pin, enable_pin
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

    def step_once(self):
        """STEP 한 펄스를 출력한다. 출력 도중 실패해도 LOW 복귀를 시도한다."""
        self.check_connection()
        if not self._enabled or self._direction is None:
            raise RuntimeError('set direction and enable before STEP')
        try:
            self._call('write', self.step_pin, 1)
            self._sleep(self._high)
        finally:
            self._call('write', self.step_pin, 0)
        self._sleep(self._low)

    def disable(self):
        """STEP LOW와 EN HIGH를 각각 시도한다. 한 출력이 실패해도 다른 출력을 시도한다."""
        self._enabled = False
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
