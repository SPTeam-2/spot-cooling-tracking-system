"""
[파일(내부 구현 코드) 목적]
실제 GPIO·pigpiod 없이 팬틸트 서보의 연결·HW·기능·목표 각도 알고리즘을 자동 검증
[코드 내용]
FakeSocket·ServoConnectionTests: 서보 명령 패킷, 명령 번호를 pigpio 모듈에서 읽는지, 펄스 범위, 통신 오류
VirtualServo: 마지막 펄스·출력 기록·오류 주입이 가능한 가상 서보 연결
ServoHardwareTests: 핀 검사, 펄스 범위, 끄기, 정리·공유 연결 유지
PanTiltFunctionTests: 축 설정 검증, 각도→펄스 환산·방향 반전, 범위 거절, 중립, 출력 실패 시 명령 각도 None
MoveToAngleTests: 각속도 제한 이동, 두 축 동시 도착, 목표 변경, STOP, 호출 지연 시 이동량 제한, 출력 오류
[플로우 간단 정리]
가상 연결·시계 준비 → HW·기능·알고리즘 생성 → 명령·update() 반복 → 펄스 기록·상태 확인

모의 검사이며 실제 서보 각도·SG90/MG90S 펄스 범위·기구 간섭 검증을 대체하지 않음
"""
from collections import deque
import math
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from src.pan_tilt.pigpio_servo_connection import PigpioServoConnection
from src.pan_tilt.hw_servo import ServoHardware
from src.pan_tilt.function_pan_tilt import AxisConfig, PanTiltFunctions
from src.pan_tilt.move_to_angle import AngleStatus as S, MoveToAngle, MoveToAngleConfig

PIGPIO = SimpleNamespace(_PI_CMD_SERVO=8, error_text=lambda code: f'error {code}')


class FakeSocket:
    def __init__(self, chunks):
        self.chunks = deque(chunks)
        self.sent = []
        self.closed = False
    def settimeout(self, value): pass
    def sendall(self, packet): self.sent.append(struct.unpack('=IIII', packet))
    def recv(self, count):
        chunk = self.chunks.popleft()
        if isinstance(chunk, Exception):
            raise chunk
        return chunk
    def close(self): self.closed = True


def response(result=0):
    return struct.pack('=IIIi', 0, 0, 0, result)


class ServoConnectionTests(unittest.TestCase):
    def connection(self, sock, module=PIGPIO):
        return PigpioServoConnection(connector=Mock(return_value=sock), pigpio_module=module)

    def test_servo_packet_uses_module_command_number(self):
        packet = response()
        sock = FakeSocket([packet[:7], packet[7:], packet])
        connection = self.connection(sock)
        connection.set_servo_pulsewidth(12, 1500)
        connection.set_servo_pulsewidth(12, 0)
        self.assertEqual(sock.sent, [(8, 12, 1500, 0), (8, 12, 0, 0)])
        other = self.connection(FakeSocket([response()]), SimpleNamespace(_PI_CMD_SERVO=99, error_text=str))
        other.set_servo_pulsewidth(13, 600)
        connection.stop()
        connection.stop()
        self.assertTrue(sock.closed)

    def test_missing_command_number_rejected(self):
        with self.assertRaises(RuntimeError):
            self.connection(FakeSocket([]), SimpleNamespace(error_text=str))

    def test_pulse_range_checked_before_sending(self):
        sock = FakeSocket([])
        connection = self.connection(sock)
        for bad in (499, 2501, 1500.0, -1, True, None):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                connection.set_servo_pulsewidth(12, bad)
        self.assertEqual(sock.sent, [])

    def test_error_and_disconnect_close_connection(self):
        connection = self.connection(FakeSocket([response(-8)]))
        with self.assertRaisesRegex(OSError, 'error -8'):
            connection.set_servo_pulsewidth(12, 1500)
        for chunks in ([b''], [b'12', TimeoutError('late')]):
            sock = FakeSocket(chunks)
            connection = self.connection(sock)
            with self.assertRaises(OSError):
                connection.set_servo_pulsewidth(12, 1500)
            self.assertFalse(connection.connected)
            with self.assertRaises(ConnectionError):
                connection.set_servo_pulsewidth(12, 1500)

    def test_invalid_timeout(self):
        for bad in (0, -1, math.inf, '1'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                PigpioServoConnection(timeout_s=bad, connector=Mock(), pigpio_module=PIGPIO)


class VirtualServo:
    """가상 서보 연결. 핀별 마지막 펄스와 전체 출력 기록"""

    def __init__(self):
        self.pulse = {}
        self.history = []
        self.fail_after = None
        self.stopped = False

    def set_servo_pulsewidth(self, pin, pulse_us):
        if self.fail_after is not None and len(self.history) >= self.fail_after:
            raise OSError('virtual servo error')
        self.pulse[pin] = pulse_us
        self.history.append((pin, pulse_us))

    def stop(self):
        self.stopped = True


class ServoHardwareTests(unittest.TestCase):
    def test_pulse_off_and_cleanup_keep_shared_connection(self):
        conn = VirtualServo()
        servo = ServoHardware(12, connection=conn)
        servo.set_pulse(1500)
        servo.off()
        servo.set_pulse(2500)
        servo.cleanup()
        servo.cleanup()
        self.assertEqual(conn.history, [(12, 1500), (12, 0), (12, 2500), (12, 0)])
        self.assertFalse(conn.stopped)
        with self.assertRaises(RuntimeError):
            servo.set_pulse(1500)

    def test_invalid_pin_and_pulse(self):
        for bad in (-1, 28, '12', 12.0):
            with self.subTest(pin=bad), self.assertRaises(ValueError):
                ServoHardware(bad, connection=VirtualServo())
        with self.assertRaises(ValueError):
            ServoHardware(18, reserved_pins=(18, 20), connection=VirtualServo())
        conn = VirtualServo()
        servo = ServoHardware(12, connection=conn)
        for bad in (0, 499, 2501, 1500.5):
            with self.subTest(pulse=bad), self.assertRaises(ValueError):
                servo.set_pulse(bad)
        self.assertEqual(conn.history, [])

    def test_cleanup_error_propagates_after_closing(self):
        conn = VirtualServo()
        servo = ServoHardware(12, connection=conn)
        conn.fail_after = 0
        with self.assertRaises(OSError):
            servo.cleanup()
        with self.assertRaises(RuntimeError):
            servo.set_pulse(1500)


def axis(**overrides):
    values = dict(center_pulse_us=1500, us_per_deg=10, min_deg=-60, max_deg=60)
    values.update(overrides)
    return AxisConfig(**values)


class PanTiltBase(unittest.TestCase):
    def build(self, pan=None, tilt=None):
        self.conn = VirtualServo()
        self.pan_hw = ServoHardware(12, connection=self.conn)
        self.tilt_hw = ServoHardware(13, connection=self.conn)
        self.pt = PanTiltFunctions(self.pan_hw, self.tilt_hw, pan or axis(),
                                   tilt or axis(center_pulse_us=1400, us_per_deg=-11, min_deg=-30, max_deg=45))


class PanTiltFunctionTests(PanTiltBase):
    def test_axis_config_validation(self):
        bad = [dict(center_pulse_us=1500.0), dict(center_pulse_us=400), dict(us_per_deg=0),
               dict(us_per_deg=math.nan), dict(min_deg=10), dict(max_deg=-10), dict(min_deg=0, max_deg=0),
               dict(max_deg=math.inf), dict(us_per_deg=20, max_deg=60)]   # 1500 + 20*60 = 2700us
        for overrides in bad:
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                axis(**overrides)

    def test_angle_to_pulse_and_direction(self):
        self.build()
        self.assertFalse(self.pt.ready)
        self.assertIsNone(self.pt.angle('pan'))
        self.pt.center()
        self.assertTrue(self.pt.ready)
        self.assertEqual(self.conn.pulse, {12: 1500, 13: 1400})
        self.pt.set_angle('pan', -45.5)
        self.pt.set_angle('tilt', 30)
        self.assertEqual(self.conn.pulse, {12: 1045, 13: 1070})   # tilt는 us_per_deg 음수로 방향 반전
        self.assertEqual((self.pt.angle('pan'), self.pt.angle('tilt')), (-45.5, 30))

    def test_rejects_out_of_range_without_output(self):
        self.build()
        self.pt.center()
        before = list(self.conn.history)
        for name, deg in (('pan', 60.1), ('pan', -61), ('tilt', 46), ('tilt', -31),
                          ('pan', math.nan), ('pan', '10'), ('pan', None)):
            with self.subTest(axis=name, deg=deg), self.assertRaises(ValueError):
                self.pt.set_angle(name, deg)
        with self.assertRaises(ValueError):
            self.pt.set_angle('roll', 0)
        self.assertEqual(self.conn.history, before)
        self.assertEqual(self.pt.angle('pan'), 0)

    def test_output_failure_clears_commanded_angle(self):
        self.build()
        self.pt.center()
        self.conn.fail_after = len(self.conn.history)
        with self.assertRaises(OSError):
            self.pt.set_angle('pan', 10)
        self.assertIsNone(self.pt.angle('pan'))
        self.assertFalse(self.pt.ready)

    def test_off_clears_angles_and_tries_both_axes(self):
        self.build()
        self.pt.center()
        self.pt.off()
        self.assertEqual(self.conn.pulse, {12: 0, 13: 0})
        self.assertFalse(self.pt.ready)
        self.pt.center()
        self.conn.fail_after = len(self.conn.history)
        with self.assertRaises(RuntimeError):
            self.pt.off()
        self.assertFalse(self.pt.ready)


class MoveToAngleTests(PanTiltBase):
    def setUp(self):
        self.build()
        self.now = 0.0
        self.mover = MoveToAngle(self.pt, MoveToAngleConfig(max_speed_deg_s=100), clock=lambda: self.now)

    def run_until_done(self, dt=0.02, limit=10000, stop_at=None):
        result = self.mover.result
        for count in range(limit):
            if result.status != S.MOVING:
                return result
            self.now += dt
            result = self.mover.update(stop_requested=count == stop_at)
        self.fail('iteration bound exceeded')

    def test_requires_center_first(self):
        with self.assertRaises(RuntimeError):
            self.mover.set_target(10, 10)
        self.assertEqual(self.conn.history, [])

    def test_moves_with_speed_limit_and_arrives_together(self):
        self.pt.center()
        self.mover.set_target(50, -20)
        pan_angles = []
        result = self.mover.result
        while result.status == S.MOVING:
            self.now += 0.02
            result = self.mover.update()
            pan_angles.append(self.pt.angle('pan'))
        self.assertEqual(result.status, S.ARRIVED)
        self.assertEqual((result.pan, result.tilt), (50, -20))
        steps = [b - a for a, b in zip([0] + pan_angles, pan_angles)]
        self.assertTrue(all(abs(step) <= 2.0 + 1e-6 for step in steps))   # 100deg/s × 0.02s
        self.assertIn(len(pan_angles), (25, 26))                            # 50도 ÷ 2도 (부동소수점 오차로 마지막 짧은 1회 가능)
        self.assertEqual(self.conn.pulse, {12: 2000, 13: 1620})

    def test_skips_updates_faster_than_interval(self):
        self.pt.center()
        self.mover.set_target(10, 0)
        sent = len(self.conn.history)
        for _ in range(5):
            self.now += 0.001
            self.mover.update()
        self.assertEqual(len(self.conn.history), sent)

    def test_late_update_limits_jump(self):
        self.pt.center()
        self.mover.set_target(60, 0)
        self.now += 3.0                                                       # 루프가 3초 멈춘 상황
        self.mover.update()
        self.assertEqual(self.pt.angle('pan'), 10.0)                          # 100deg/s × (0.02 × 5)

    def test_already_there_arrives_without_output(self):
        self.pt.center()
        sent = len(self.conn.history)
        self.assertEqual(self.mover.set_target(0, 0).status, S.ARRIVED)
        self.assertEqual(len(self.conn.history), sent)

    def test_rejected_target_keeps_current_move(self):
        self.pt.center()
        self.mover.set_target(40, 0)
        for _ in range(5):
            self.now += 0.02
            self.mover.update()
        with self.assertRaises(ValueError):
            self.mover.set_target(90, 0)
        self.assertEqual(self.mover.result.target_pan, 40)
        self.assertEqual(self.run_until_done().status, S.ARRIVED)
        self.assertEqual(self.pt.angle('pan'), 40)

    def test_new_target_while_moving(self):
        self.pt.center()
        self.mover.set_target(50, 40)
        for _ in range(10):
            self.now += 0.02
            self.mover.update()
        self.assertAlmostEqual(self.pt.angle('pan'), 20)
        self.mover.set_target(-10, 0)
        self.now += 0.02
        self.mover.update()
        self.assertAlmostEqual(self.pt.angle('pan'), 18)                      # 그 자리에서 반대로
        result = self.run_until_done()
        self.assertEqual((result.status, result.pan, result.tilt), (S.ARRIVED, -10, 0))

    def test_stop_keeps_last_pulse(self):
        self.pt.center()
        self.mover.set_target(50, 0)
        result = self.run_until_done(stop_at=5)
        self.assertEqual(result.status, S.CANCELLED)
        held = dict(self.conn.pulse)
        self.assertNotEqual(held[12], 0)
        self.now += 1
        self.mover.update()
        self.assertEqual(self.conn.pulse, held)
        self.assertTrue(self.pt.ready)
        self.mover.set_target(0, 0)
        self.assertEqual(self.run_until_done().status, S.ARRIVED)

    def test_output_error_fails_and_turns_off(self):
        self.pt.center()
        self.mover.set_target(50, 0)
        self.conn.fail_after = len(self.conn.history) + 3
        result = self.run_until_done()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('servo output failed', result.error)
        self.assertFalse(self.pt.ready)
        with self.assertRaises(RuntimeError):
            self.mover.set_target(0, 0)

    def test_invalid_config(self):
        for values in (dict(max_speed_deg_s=0), dict(max_speed_deg_s=math.nan),
                       dict(max_speed_deg_s=10, update_interval_s=0), dict(max_speed_deg_s='10')):
            with self.subTest(values=values), self.assertRaises(ValueError):
                MoveToAngleConfig(**values)


if __name__ == '__main__':
    unittest.main()
