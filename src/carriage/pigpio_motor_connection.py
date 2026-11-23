"""
[파일(내부 구현 코드) 목적]
모터 출력용 로컬 pigpiod 연결 계층 (pigpio 라이브러리 사용)
[코드 내용]
PigpioMotorConnection: 127.0.0.1:8888 연결, 출력 모드·레벨, STEP PWM 시작·정지, 상승 에지 콜백, 현재 tick
[플로우 간단 정리]
연결 생성 → set_mode()·write() → set_pwm()으로 STEP 펄스 → rising_edge_callback()으로 펄스 세기 → pwm_off() → stop()

펄스 방식 A(PWM + 콜백 카운트)를 위해 소켓 직접 통신 대신 pigpio 라이브러리를 사용 (2026-11-23 결정)
- PWM 주파수는 pigpio 기본 샘플링(5µs)에서 정해진 값만 가능, 요청값과 실제값이 다르면 오류
- pigpio 라이브러리는 명령별 응답 제한 시간을 지원하지 않음 → timeout_s는 값 검사만 하고 명령에 적용하지 않음
- 콜백은 pigpiod 알림을 별도 스레드에서 받음 (알림 지연은 현장 점검에서 1~2ms 수준)
- 연결 종료만으로 출력 복구는 보장되지 않음 → 종료 전 상위 계층에서 STEP LOW·EN HIGH
"""
import math

PWM_DUTY_HALF = 128   # 0~255 중 절반 = STEP HIGH·LOW 같은 길이


class PigpioMotorConnection:
    """로컬 Pi 전용 연결. 환경변수 PIGPIO_ADDR/PIGPIO_PORT를 쓰지 않고 같은 Pi만 제어한다."""

    def __init__(self, timeout_s=0.25, *, pigpio_module=None):
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('통신 제한 시간은 유한한 양수여야 합니다.')
        if pigpio_module is None:
            import pigpio as pigpio_module
        self._pigpio = pigpio_module
        self.timeout_s = timeout_s
        self._pi = pigpio_module.pi('127.0.0.1', 8888)
        if not self._pi.connected:
            raise ConnectionError('pigpiod 연결 실패: sudo pigpiod 먼저 실행')
        self._closed = False

    @property
    def connected(self):
        return not self._closed and bool(self._pi.connected)

    def _check(self):
        if not self.connected:
            raise ConnectionError('pigpiod 연결이 닫혔습니다. 새 연결이 필요합니다.')

    @staticmethod
    def _ok(value, name):
        if type(value) is not int or value < 0:
            raise OSError(f'pigpio {name} failed: {value!r}')
        return value

    def set_mode(self, pin, mode):
        if mode != self._pigpio.OUTPUT:
            raise ValueError('모터 연결은 출력 모드만 지원합니다.')
        self._check()
        self._ok(self._pi.set_mode(pin, mode), 'set_mode')
        return 0

    def write(self, pin, level):
        """레벨 출력. 해당 핀의 PWM이 켜져 있으면 pigpiod가 PWM을 끈다."""
        self._check()
        self._ok(self._pi.write(pin, level), 'write')
        return 0

    def set_pwm(self, pin, hz):
        """주파수를 바꾸고 duty 50%로 STEP 펄스 출력. 실제 주파수가 요청과 다르면 오류"""
        self._check()
        actual = self._ok(self._pi.set_PWM_frequency(pin, hz), 'set_PWM_frequency')
        if actual != hz:
            raise OSError(f'PWM 주파수 {hz}Hz 불가 (pigpio가 {actual}Hz로 설정)')
        self._ok(self._pi.set_PWM_dutycycle(pin, PWM_DUTY_HALF), 'set_PWM_dutycycle')
        return 0

    def pwm_off(self, pin):
        """duty 0으로 STEP 펄스 정지 (핀은 LOW 유지)"""
        self._check()
        self._ok(self._pi.set_PWM_dutycycle(pin, 0), 'set_PWM_dutycycle')
        return 0

    def rising_edge_callback(self, pin, func):
        """func(gpio, level, tick)을 상승 에지마다 호출. 반환 객체의 cancel()로 해제"""
        self._check()
        return self._pi.callback(pin, self._pigpio.RISING_EDGE, func)

    def current_tick(self):
        return self._pi.get_current_tick()

    def stop(self):
        """연결 종료는 GPIO 출력 복구를 보장하지 않는다. 반복 호출 허용"""
        if not self._closed:
            self._closed = True
            self._pi.stop()
