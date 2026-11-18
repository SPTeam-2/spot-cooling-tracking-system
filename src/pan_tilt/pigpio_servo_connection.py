"""
[파일(내부 구현 코드) 목적]
서보 펄스 출력용 로컬 pigpiod 통신을 수행하고 명령 응답 대기 시간을 제한하는 연결 계층
[코드 내용]
PigpioServoConnection: 127.0.0.1:8888 연결, 서보 펄스폭 명령 송신, 분할 응답 수신
명령 전체 송수신 제한 시간 검사 / 응답 유실·시간 초과 시 연결 폐기 및 재사용 차단
[플로우 간단 정리]
연결 생성 → set_servo_pulsewidth() → 제한 시간 내 응답 수신 → 오류 처리 또는 stop()

명령 번호는 직접 적지 않고 Pi에 설치된 pigpio 모듈의 상수(_PI_CMD_SERVO)를 읽어 사용
펄스폭: 0 = 출력 끄기, 500~2500us = 서보 펄스 (pigpiod 매뉴얼 기준) / 펄스는 바꿀 때까지 계속 출력
캐리지용 src/carriage/pigpio_motor_connection.py와 송수신 방식이 같음 (브랜치 독립을 위해 분리, 캐리지 PR 머지 후 공용화 검토)
"""
import math
import socket
import struct
import time

PULSE_OFF = 0
PULSE_MIN_US = 500
PULSE_MAX_US = 2500


class PigpioServoConnection:
    """로컬 Pi 전용 연결. 응답 유실 후에는 연결을 닫고 명령 재사용을 금지"""

    def __init__(self, timeout_s=0.25, *, connector=socket.create_connection,
                 clock=time.monotonic, pigpio_module=None):
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('통신 제한 시간은 유한한 양수여야 합니다.')
        if pigpio_module is None:
            import pigpio as pigpio_module
        command = getattr(pigpio_module, '_PI_CMD_SERVO', None)
        if type(command) is not int:
            raise RuntimeError('설치된 pigpio 모듈에서 서보 명령 번호(_PI_CMD_SERVO)를 찾지 못했습니다.')
        self._pigpio = pigpio_module
        self._servo_command = command
        self._timeout = timeout_s
        self._clock = clock
        self.connected = False
        # 환경변수 PIGPIO_ADDR/PIGPIO_PORT를 사용하지 않음. 같은 Pi만 제어
        self._socket = connector(('127.0.0.1', 8888), timeout=timeout_s)
        self.connected = True

    def _command(self, command, pin, value):
        if not self.connected:
            raise ConnectionError('pigpiod 연결이 닫혔습니다. 새 연결이 필요합니다.')
        deadline = self._clock() + self._timeout
        try:
            self._socket.settimeout(self._timeout)
            self._socket.sendall(struct.pack('=IIII', command, pin, value, 0))
            response = bytearray()
            while len(response) < 16:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise TimeoutError('pigpiod 명령 응답 시간이 초과됐습니다.')
                self._socket.settimeout(remaining)
                chunk = self._socket.recv(16 - len(response))
                if not chunk:
                    raise ConnectionError('pigpiod가 응답 도중 연결을 종료했습니다.')
                response.extend(chunk)
            result = struct.unpack('=IIIi', response)[3]
        except (OSError, ValueError, struct.error):
            # 지연 응답을 다음 명령의 응답으로 잘못 해석하지 않도록 연결 폐기
            self.stop()
            raise
        if result < 0:
            raise OSError(f'pigpiod 명령 실패: {self._pigpio.error_text(result)} ({result})')
        return result

    def set_servo_pulsewidth(self, pin, pulse_us):
        """0이면 출력 끄기, 500~2500이면 해당 펄스폭으로 계속 출력"""
        if type(pulse_us) is not int or not (pulse_us == PULSE_OFF or PULSE_MIN_US <= pulse_us <= PULSE_MAX_US):
            raise ValueError('pulse_us must be 0 or an integer in 500..2500')
        return self._command(self._servo_command, pin, pulse_us)

    def stop(self):
        """연결 종료는 서보 출력 정지를 보장하지 않음. 종료 전에 펄스 0을 먼저 보냄"""
        if self.connected:
            self.connected = False
            self._socket.close()
