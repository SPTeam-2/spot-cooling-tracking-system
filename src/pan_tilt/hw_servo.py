"""
[파일(내부 구현 코드) 목적]
서보 1개에 펄스폭 신호를 출력하고 끄는 HW 계층 (pan·tilt에 같은 클래스를 각각 사용)
[코드 내용]
ServoHardware: BCM 핀 검사, 펄스폭(us) 출력, 출력 끄기, 종료 정리
각도 환산·범위 제한은 상위 계층 담당 / 기본 연결은 대기 제한이 있는 로컬 pigpiod 연결 사용
[플로우 간단 정리]
객체 생성 → 핀 검증 → set_pulse() 반복 → off() → cleanup()

부품: pan(좌우) SG90, tilt(상하) MG90S / 전원은 강압 모듈 5V, GND는 Pi와 공통
피드백이 없어 실제 각도는 알 수 없음 / 펄스 0이면 서보 힘이 풀림
통신 실패 시 모의 장치로 자동 전환하지 않음 / 주입받은 공유 연결은 종료하지 않음
참고: 개인 노션 #12, #코드 구조 관련(1장 파일 이름, 6장 팬틸트) / pins.yaml servo.pan·servo.tilt
"""
from .pigpio_servo_connection import PULSE_MAX_US, PULSE_MIN_US, PULSE_OFF


class ServoHardware:
    """서보 신호 출력 담당. 주입받은 공유 연결은 종료하지 않음"""

    def __init__(self, pin, *, reserved_pins=(), connection=None, command_timeout_s=0.25):
        if type(pin) is not int or not 0 <= pin <= 27:
            raise ValueError('BCM pin must be an integer in 0..27')
        if pin in reserved_pins:
            raise ValueError(f'pin {pin} is reserved by another part')
        self.pin = pin
        self._owned = connection is None
        if connection is None:
            from .pigpio_servo_connection import PigpioServoConnection
            connection = PigpioServoConnection(timeout_s=command_timeout_s)
        self._conn = connection
        self._closed = False
        self.pulse_us = PULSE_OFF

    def set_pulse(self, pulse_us):
        """500~2500us 펄스 출력. 바꿀 때까지 계속 유지"""
        if self._closed:
            raise RuntimeError('servo hardware closed')
        if type(pulse_us) is not int or not PULSE_MIN_US <= pulse_us <= PULSE_MAX_US:
            raise ValueError('pulse_us must be an integer in 500..2500')
        self._conn.set_servo_pulsewidth(self.pin, pulse_us)
        self.pulse_us = pulse_us

    def off(self):
        """펄스 출력 끄기 (서보 힘 풀림)"""
        if self._closed:
            return
        self._conn.set_servo_pulsewidth(self.pin, PULSE_OFF)
        self.pulse_us = PULSE_OFF

    def cleanup(self):
        """출력 끄기를 시도한 뒤 직접 만든 연결만 종료. 반복 호출 허용"""
        if self._closed:
            return
        error = None
        try:
            self.off()
        except Exception as exc:
            error = exc
        self._closed = True
        if self._owned:
            self._conn.stop()
        if error is not None:
            raise error
