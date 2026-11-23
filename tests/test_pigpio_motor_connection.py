"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 pigpio 라이브러리 기반 모터 연결과 방식 A(PWM + 콜백 카운트) 전체 흐름을 자동 검증
[코드 내용]
FakePigpio·FakePi: pigpio.pi 흉내 (PWM 주파수 표, 콜백 등록, emit()으로 STEP 상승 에지 발생)
ConnectionTests: 연결 실패, 명령 실패, 허용되지 않는 주파수, 종료 후 명령 차단
PulseChainTests: 연결 + StepperHardware + StepperFunctions로 정확한 카운트, 가감속 순서, 리미트·STOP, 초과 스텝 반영
[플로우 간단 정리]
가짜 pigpio 준비 → 연결·모터 계층 생성 → 이동 요청 → emit()으로 펄스 발생 → update() → 결과 확인

실제 pigpiod·GPIO·콜백 지연을 구동하지 않음 / 정지 지연·탈조는 Pi 실물 시험으로 확인
"""
import unittest
from unittest.mock import Mock

from src.carriage.pigpio_motor_connection import PigpioMotorConnection
from src.carriage.hw_stepper import StepperHardware
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions, StepperReason as R

ALLOWED = (10, 20, 40, 50, 80, 100, 160, 200, 250, 320, 400, 500, 800, 1000, 1600, 2000)


class FakeCallback:
    def __init__(self, pi, pin, func):
        self.pi, self.pin, self.func = pi, pin, func
        self.cancelled = False

    def cancel(self):
        self.cancelled = True
        if self in self.pi.callbacks:
            self.pi.callbacks.remove(self)


class FakePi:
    def __init__(self, host, port, connected=True):
        self.address = (host, port)
        self.connected = connected
        self.calls = []
        self.callbacks = []
        self.freq = {}
        self.duty = {}
        self.fail = set()
        self.tick = 0

    def _maybe_fail(self, name):
        if name in self.fail:
            return -99
        return 0

    def set_mode(self, pin, mode):
        self.calls.append(('mode', pin, mode))
        return self._maybe_fail('set_mode')

    def write(self, pin, level):
        self.calls.append(('write', pin, level))
        self.duty[pin] = 0                  # pigpiod: write는 PWM을 끈다
        return self._maybe_fail('write')

    def set_PWM_frequency(self, pin, hz):
        self.calls.append(('freq', pin, hz))
        if 'freq' in self.fail:
            return -99
        actual = min(ALLOWED, key=lambda f: abs(f - hz))
        self.freq[pin] = actual
        return actual

    def set_PWM_dutycycle(self, pin, duty):
        self.calls.append(('duty', pin, duty))
        self.duty[pin] = duty
        return self._maybe_fail('duty')

    def callback(self, pin, edge, func):
        cb = FakeCallback(self, pin, func)
        self.callbacks.append(cb)
        return cb

    def get_current_tick(self):
        return self.tick

    def stop(self):
        self.calls.append(('stop',))
        self.connected = False

    def running(self, pin):
        return self.duty.get(pin, 0) > 0

    def emit(self, pin, n, late=0):
        """PWM이 켜져 있는 동안 상승 에지 n개 발생. late = PWM이 꺼진 뒤 늦게 도착하는 알림 수"""
        sent = 0
        while sent < n and self.running(pin):
            self.tick += 1
            for cb in list(self.callbacks):
                if cb.pin == pin:
                    cb.func(pin, 1, self.tick)
            sent += 1
        for _ in range(late):
            for cb in list(self.callbacks):
                if cb.pin == pin:
                    cb.func(pin, 1, self.tick)
        return sent


def fake_module(connected=True):
    module = Mock()
    module.OUTPUT, module.RISING_EDGE = 1, 0
    module.made = []

    def make(host, port):
        pi = FakePi(host, port, connected)
        module.made.append(pi)
        return pi
    module.pi.side_effect = make
    return module


class ConnectionTests(unittest.TestCase):
    def test_local_connection_and_commands(self):
        module = fake_module()
        conn = PigpioMotorConnection(pigpio_module=module)
        pi = module.made[0]
        self.assertEqual(pi.address, ('127.0.0.1', 8888))
        self.assertEqual(conn.set_mode(20, 1), 0)
        self.assertEqual(conn.write(20, 0), 0)
        self.assertEqual(conn.set_pwm(20, 250), 0)
        self.assertEqual((pi.freq[20], pi.duty[20]), (250, 128))
        conn.pwm_off(20)
        self.assertEqual(pi.duty[20], 0)
        conn.stop()
        conn.stop()
        self.assertEqual(pi.calls.count(('stop',)), 1)
        with self.assertRaises(ConnectionError):
            conn.write(20, 0)

    def test_connect_failure_and_invalid_timeout(self):
        with self.assertRaises(ConnectionError):
            PigpioMotorConnection(pigpio_module=fake_module(connected=False))
        for timeout in (0, -1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                PigpioMotorConnection(timeout, pigpio_module=fake_module())

    def test_unsupported_frequency_and_errors(self):
        module = fake_module()
        conn = PigpioMotorConnection(pigpio_module=module)
        with self.assertRaisesRegex(OSError, '255Hz'):
            conn.set_pwm(20, 255)                     # pigpio가 250Hz로 바꿈 → 오류
        module.made[0].fail.add('write')
        with self.assertRaises(OSError):
            conn.write(20, 1)
        with self.assertRaises(ValueError):
            conn.set_mode(20, 0)                      # 입력 모드 금지


class PulseChainTests(unittest.TestCase):
    def setUp(self):
        module = fake_module()
        self.conn = PigpioMotorConnection(pigpio_module=module)
        self.pi = module.made[0]
        self.hw = StepperHardware(20, 21, 16, positive_dir_level=1, pi=self.conn, sleep=lambda _: None)
        self.reader = Mock()
        self.reader.read_levels.return_value = (1, 1)
        self.now = 0.0
        self.motor = StepperFunctions(self.hw, LimitSwitchFunctions(self.reader),
                                      max_speed_steps_s=250, clock=lambda: self.now)

    def freqs(self):
        return [c[2] for c in self.pi.calls if c[0] == 'freq']

    def test_auto_mode_is_pwm(self):
        self.assertEqual(self.motor.pulse_mode, 'pwm')

    def test_exact_count_with_ramp(self):
        result = self.motor.start_move(1, 1600, speed_steps_s=250)
        self.assertEqual(result.reason, R.RUNNING)
        self.assertEqual(self.pi.emit(20, 2000), 1600)      # 1600개에서 PWM이 꺼짐
        result = self.motor.update()
        self.assertEqual((result.reason, result.completed_steps), (R.COMPLETED, 1600))
        self.assertEqual(self.freqs(), [160, 200, 250, 200, 160])
        self.assertEqual(self.pi.callbacks, [])               # 콜백 해제
        self.assertIn(('write', 16, 0), self.pi.calls)        # 정상 완료는 EN 유지(토크 유지)

    def test_late_pulses_counted_as_overshoot(self):
        self.motor.start_move(1, 400, speed_steps_s=250)
        self.pi.emit(20, 400, late=1)                         # 정지 명령 뒤 늦게 도착한 1개
        result = self.motor.update()
        self.assertEqual((result.reason, result.completed_steps), (R.COMPLETED, 401))

    def test_short_move_uses_low_speed_only(self):
        self.motor.start_move(-1, 30, speed_steps_s=250)
        self.pi.emit(20, 100)
        self.assertEqual(self.motor.update().completed_steps, 30)
        self.assertEqual(self.freqs(), [160])

    def test_limit_stops_pwm(self):
        self.motor.start_move(1, 1600, speed_steps_s=200)
        self.pi.emit(20, 300)
        self.reader.read_levels.return_value = (1, 0)         # 오른쪽 눌림
        result = self.motor.update()
        self.assertEqual((result.reason, result.completed_steps), (R.RIGHT_LIMIT, 300))
        self.assertFalse(self.pi.running(20))
        self.assertEqual(self.pi.emit(20, 10), 0)

    def test_stop_and_no_auto_resume(self):
        self.motor.start_move(1, 1600, speed_steps_s=250)
        self.pi.emit(20, 100)
        self.assertEqual(self.motor.update(stop_requested=True).reason, R.STOPPED)
        self.assertEqual(self.motor.result.completed_steps, 100)
        self.now = 100
        self.motor.update()
        self.assertFalse(self.pi.running(20))

    def test_timeout_is_fault(self):
        self.motor.start_move(1, 1600, speed_steps_s=250)
        self.pi.emit(20, 10)
        self.now = 1000                                       # 예상 시간 + 여유를 넘김
        result = self.motor.update()
        self.assertEqual(result.reason, R.FAULT)
        self.assertTrue(result.position_uncertain)
        self.assertFalse(self.pi.running(20))
        self.assertIn(('write', 16, 1), self.pi.calls)        # EN 해제

    def test_callback_command_error_is_fault(self):
        self.motor.start_move(1, 1600, speed_steps_s=250)
        self.pi.fail.add('freq')
        self.pi.emit(20, 100)                                 # 40번째에서 200Hz 전환 실패
        result = self.motor.update()
        self.assertEqual(result.reason, R.FAULT)
        self.assertTrue(result.position_uncertain)
        self.assertFalse(self.pi.running(20))

    def test_rejects_non_pwm_speed(self):
        for speed in (150, 230):
            with self.assertRaises(ValueError):
                self.motor.start_move(1, 100, speed_steps_s=speed)
        self.assertNotIn(('write', 16, 0), self.pi.calls)                # EN을 켜지 않음

    def test_start_failure_cleans_up(self):
        self.pi.fail.add('freq')
        result = self.motor.start_move(1, 100, speed_steps_s=250)
        self.assertEqual(result.reason, R.FAULT)
        self.assertTrue(result.position_uncertain)
        self.assertEqual(self.pi.callbacks, [])
        self.assertFalse(self.pi.running(20))


class PulseHardwareTests(unittest.TestCase):
    def setUp(self):
        module = fake_module()
        self.pi_conn = PigpioMotorConnection(pigpio_module=module)
        self.pi = module.made[0]
        self.hw = StepperHardware(20, 21, 16, positive_dir_level=1, pi=self.pi_conn, sleep=lambda _: None)

    def test_requires_direction_enable_and_valid_plan(self):
        with self.assertRaises(RuntimeError):
            self.hw.start_pulses(10, [(0, 160)])
        self.hw.set_direction(1)
        self.hw.enable()
        for total, plan in ((0, [(0, 160)]), (10, []), (10, [(1, 160)]), (10, [(0, 160), (10, 200)]),
                            (10, [(0, 160), (0, 200)]), (10, [(0, 0)])):
            with self.assertRaises(ValueError):
                self.hw.start_pulses(total, plan)
        self.assertEqual(self.pi.callbacks, [])

    def test_no_direction_change_or_step_once_while_running(self):
        self.hw.set_direction(1)
        self.hw.enable()
        self.hw.start_pulses(100, [(0, 160)])
        with self.assertRaises(RuntimeError):
            self.hw.set_direction(-1)
        with self.assertRaises(RuntimeError):
            self.hw.step_once()
        with self.assertRaises(RuntimeError):
            self.hw.start_pulses(100, [(0, 160)])
        self.pi.emit(20, 20)
        self.assertEqual(self.hw.finish_pulses(), 20)
        self.hw.set_direction(-1)                              # 끝난 뒤에는 가능

    def test_cleanup_stops_pwm_and_cancels_callback(self):
        self.hw.set_direction(1)
        self.hw.enable()
        self.hw.start_pulses(100, [(0, 250)])
        self.hw.cleanup()
        self.assertFalse(self.pi.running(20))
        self.assertEqual(self.pi.callbacks, [])
        self.assertEqual(self.pi.calls[-1], ('write', 16, 1))
        self.assertNotIn(('stop',), self.pi.calls)            # 주입받은 공유 연결은 종료하지 않음
