"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 pigpiod 명령 형식과 통신 오류·시간 초과 처리를 자동 검증
[코드 내용]
FakeSocket·ConnectionTests: 명령 패킷, 분할 응답, 음수 오류 응답, 연결 종료 검사
전체 응답 제한 시간, 로컬 소켓 무응답, 실패 연결 재사용 금지, 모터 FAULT 연결 검사
[플로우 간단 정리]
가상 또는 로컬 소켓 준비 → 명령 송신 → 응답·오류 주입 → 연결 폐기·추가 출력 차단 확인

실제 pigpiod·GPIO를 구동하지 않음 / 로컬 소켓 시험은 실행 환경의 네트워크 권한이 필요할 수 있음
"""
from collections import deque
import socket
import struct
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from src.carriage.pigpio_motor_connection import PigpioMotorConnection
from src.carriage.hw_stepper import StepperHardware
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions, StepperReason

PIGPIO = SimpleNamespace(OUTPUT=1, error_text=lambda code: f'error {code}')


class FakeSocket:
    def __init__(self, chunks):
        self.chunks = deque(chunks)
        self.sent = []
        self.closed = False
    def settimeout(self, value):
        self.timeout = value
    def sendall(self, packet):
        self.sent.append(struct.unpack('=IIII', packet))
    def recv(self, count):
        chunk = self.chunks.popleft()
        if isinstance(chunk, Exception):
            raise chunk
        return chunk
    def close(self):
        self.closed = True


def response(result=0):
    return struct.pack('=IIIi', 0, 0, 0, result)


class ConnectionTests(unittest.TestCase):
    def connection(self, sock, **kwargs):
        connector = Mock(return_value=sock)
        connection = PigpioMotorConnection(connector=connector, pigpio_module=PIGPIO, **kwargs)
        connector.assert_called_once_with(('127.0.0.1', 8888), timeout=kwargs.get('timeout_s', .25))
        return connection

    def test_protocol_and_fragmented_response(self):
        packet = response()
        sock = FakeSocket([packet[:5], packet[5:], packet])
        connection = self.connection(sock)
        self.assertEqual(connection.set_mode(20, 1), 0)
        self.assertEqual(connection.write(20, 0), 0)
        self.assertEqual(sock.sent, [(0, 20, 1, 0), (4, 20, 0, 0)])
        connection.stop()
        connection.stop()
        self.assertTrue(sock.closed)

    def test_negative_result_is_error(self):
        connection = self.connection(FakeSocket([response(-2)]))
        with self.assertRaisesRegex(OSError, 'error -2'):
            connection.write(20, 0)

    def test_disconnect_and_partial_reply_close_connection(self):
        for chunks in ([b''], [b'123', TimeoutError('late')]):
            sock = FakeSocket(chunks)
            connection = self.connection(sock)
            with self.assertRaises(OSError):
                connection.write(20, 1)
            self.assertFalse(connection.connected)
            self.assertTrue(sock.closed)
            with self.assertRaises(ConnectionError):
                connection.write(20, 0)
            self.assertEqual(len(sock.sent), 1)

    def test_total_deadline_not_reset_by_fragments(self):
        sock = FakeSocket([b'1', b'2'])
        clock = Mock(side_effect=[0, .01, .2])
        connection = self.connection(sock, timeout_s=.1, clock=clock)
        with self.assertRaises(TimeoutError):
            connection.write(20, 1)
        self.assertTrue(sock.closed)

    def test_real_socket_without_reply_times_out(self):
        client, daemon = socket.socketpair()
        try:
            connection = self.connection(client, timeout_s=.03)
            before = time.monotonic()
            with self.assertRaises(TimeoutError):
                connection.write(20, 0)
            self.assertLess(time.monotonic() - before, 1)
            self.assertFalse(connection.connected)
        finally:
            client.close()
            daemon.close()

    def test_connect_failure_and_invalid_timeout(self):
        connector = Mock(side_effect=TimeoutError('connect'))
        with self.assertRaises(TimeoutError):
            PigpioMotorConnection(connector=connector, pigpio_module=PIGPIO)
        for timeout in (0, -1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                PigpioMotorConnection(timeout, connector=connector, pigpio_module=PIGPIO)
        self.assertEqual(connector.call_count, 1)

    def test_step_timeout_latches_fault_and_no_more_commands(self):
        # 초기화 6회·방향 1회·EN 1회 이후 STEP HIGH의 응답이 유실되는 상황.
        sock = FakeSocket([response()] * 8 + [TimeoutError('STEP reply lost')])
        connection = self.connection(sock)
        hardware = StepperHardware(20, 21, 16, positive_dir_level=1, pi=connection, sleep=lambda _: None)
        reader = Mock()
        reader.read_levels.return_value = (1, 1)
        motor = StepperFunctions(hardware, LimitSwitchFunctions(reader), max_speed_steps_s=100)
        motor.start_move(1, 2, speed_steps_s=10)
        result = motor.update()
        self.assertEqual(result.reason, StepperReason.FAULT)
        self.assertTrue(result.position_uncertain)
        self.assertEqual(result.completed_steps, 0)
        count = len(sock.sent)
        motor.update()
        self.assertEqual(len(sock.sent), count)
        with self.assertRaises(OSError):
            hardware.cleanup()
