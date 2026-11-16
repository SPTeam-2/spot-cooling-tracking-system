"""
[파일(내부 구현 코드) 목적]
모터 출력용 로컬 pigpiod 통신을 수행하고 연결·명령 응답 대기 시간을 제한하는 연결 계층
[코드 내용]
PigpioMotorConnection: 127.0.0.1:8888 연결, 출력 모드·레벨 명령 송신, 분할 응답 수신
명령 전체 송수신 제한 시간 검사 / 응답 유실·시간 초과 시 연결 폐기 및 재사용 차단
[플로우 간단 정리]
연결 생성 → set_mode() 또는 write() → 제한 시간 내 응답 수신 → 오류 처리 또는 stop()

pigpiod 소켓 명령 MODES(0)·WRITE(4)만 사용 / 콜백·PWM·wave는 사용하지 않음
Python pigpio의 비공개 소켓 속성을 변경하지 않음 / 연결 종료만으로 출력 복구는 보장되지 않음
프로토콜 근거: https://github.com/joan2937/pigpio/blob/v78/pigpio.py
"""
import math
import socket
import struct
import time


class PigpioMotorConnection:
    """로컬 Pi 전용 연결. 응답 유실 후에는 연결을 닫고 명령 재사용을 금지한다."""

    def __init__(self, timeout_s=0.25, *, connector=socket.create_connection,
                 clock=time.monotonic, pigpio_module=None):
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('통신 제한 시간은 유한한 양수여야 합니다.')
        if pigpio_module is None:
            import pigpio as pigpio_module
        self._pigpio = pigpio_module
        self._timeout = timeout_s
        self._clock = clock
        self.connected = False
        # 환경변수 PIGPIO_ADDR/PIGPIO_PORT를 사용하지 않는다. 리미트와 같은 Pi만 제어한다.
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
            # 지연 응답을 다음 명령의 응답으로 잘못 해석하지 않도록 연결을 폐기한다.
            self.stop()
            raise
        if result < 0:
            raise OSError(f'pigpiod 명령 실패: {self._pigpio.error_text(result)} ({result})')
        return result

    def set_mode(self, pin, mode):
        if mode != self._pigpio.OUTPUT:
            raise ValueError('모터 연결은 출력 모드만 지원합니다.')
        return self._command(0, pin, mode)

    def write(self, pin, level):
        return self._command(4, pin, level)

    def stop(self):
        """연결 종료는 GPIO 출력 복구를 보장하지 않는다."""
        if self.connected:
            self.connected = False
            self._socket.close()
