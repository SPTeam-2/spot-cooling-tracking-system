"""
[파일(내부 구현 코드) 목적]
pan(좌우)·tilt(상하) 두 축의 명령 각도·중립·허용 범위를 관리하는 기능 계층
[코드 내용]
AxisConfig: 축별 중립 펄스(0도), 1도당 펄스 변화량, 허용 각도 범위 검증
PanTiltFunctions: 각도 → 펄스 환산·범위 거절·명령 각도 기록 / center()로 중립 / off()로 출력 끄기
[플로우 간단 정리]
축·각도 입력 → 값·범위 검사 → 펄스 환산 → ServoHardware.set_pulse() → 명령 각도 기록

각도 기준: 0도 = 중립(팬이 정면을 보는 위치), 부호 방향은 us_per_deg 부호로 맞춤
피드백이 없으므로 기록하는 값은 "명령 각도"이며 실측 각도가 아님 / 첫 명령 전 각도는 None
첫 명령 시 서보는 이전 위치와 관계없이 그 각도로 바로 이동 → center()로 시작 권장
허용 범위는 #7-3 실물 간섭 확인 후 확정 / 레일 리미트 스위치는 서보 각도 제한 센서가 아님
GPIO 직접 접근 없음 (ServoHardware를 전달받아 사용)
참고: 개인 노션 #12, #7-3, #코드 구조 관련(3장 계층별 책임, 6장 팬틸트)
"""
from dataclasses import dataclass
import math

from .pigpio_servo_connection import PULSE_MAX_US, PULSE_MIN_US

AXES = ('pan', 'tilt')


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class AxisConfig:
    """실측 전 값은 기본값을 두지 않음. 호출자가 명시"""
    center_pulse_us: int    # 0도(중립)일 때 펄스
    us_per_deg: float       # 1도당 펄스 변화량, 음수면 방향 반전
    min_deg: float          # 허용 범위 시작 (0 이하)
    max_deg: float          # 허용 범위 끝 (0 이상)

    def __post_init__(self):
        if type(self.center_pulse_us) is not int or not PULSE_MIN_US <= self.center_pulse_us <= PULSE_MAX_US:
            raise ValueError('center_pulse_us must be an integer in 500..2500')
        if not _finite(self.us_per_deg) or self.us_per_deg == 0:
            raise ValueError('us_per_deg must be finite and non-zero')
        if not _finite(self.min_deg) or not _finite(self.max_deg):
            raise ValueError('angle range must be finite')
        if not self.min_deg <= 0 <= self.max_deg or self.min_deg == self.max_deg:
            raise ValueError('angle range must include 0 (center) and be non-empty')
        for deg in (self.min_deg, self.max_deg):
            if not PULSE_MIN_US <= self.pulse_for(deg) <= PULSE_MAX_US:
                raise ValueError(f'{deg}deg maps outside 500..2500us')

    def pulse_for(self, deg):
        """각도 → 정수 펄스(us), 반올림"""
        return int(round(self.center_pulse_us + self.us_per_deg * deg))


class PanTiltFunctions:
    """두 축 명령 각도의 단일 관리 창구. 단일 스레드에서 호출"""

    def __init__(self, pan_hw, tilt_hw, pan_config, tilt_config):
        self._hw = {'pan': pan_hw, 'tilt': tilt_hw}
        self.config = {'pan': pan_config, 'tilt': tilt_config}
        self._angle = {'pan': None, 'tilt': None}

    def angle(self, axis):
        """마지막으로 명령한 각도. 첫 명령 전이나 off() 이후에는 None"""
        return self._angle[self._axis(axis)]

    @property
    def ready(self):
        """두 축 모두 명령 각도가 있는 상태"""
        return all(value is not None for value in self._angle.values())

    @staticmethod
    def _axis(axis):
        if axis not in AXES:
            raise ValueError("axis must be 'pan' or 'tilt'")
        return axis

    def check_angle(self, axis, deg):
        """허용 범위 검사만 수행. 범위 밖·NaN이면 ValueError"""
        config = self.config[self._axis(axis)]
        if not _finite(deg):
            raise ValueError(f'{axis} angle must be a finite number')
        if not config.min_deg <= deg <= config.max_deg:
            raise ValueError(f'{axis} {deg}deg outside range {config.min_deg}~{config.max_deg}deg')

    def set_angle(self, axis, deg):
        """검사 → 펄스 출력 → 명령 각도 기록. 출력 실패 시 명령 각도를 None으로 (실제 위치 불확실)"""
        self.check_angle(axis, deg)
        try:
            self._hw[axis].set_pulse(self.config[axis].pulse_for(deg))
        except Exception:
            self._angle[axis] = None
            raise
        self._angle[axis] = deg

    def center(self):
        """두 축을 중립(0도)으로"""
        for axis in AXES:
            self.set_angle(axis, 0)

    def off(self):
        """두 축 출력 끄기. 서보 힘이 풀려 위치를 알 수 없으므로 명령 각도도 None. 오류가 나도 두 축 모두 시도"""
        errors = []
        for axis in AXES:
            self._angle[axis] = None
            try:
                self._hw[axis].off()
            except Exception as exc:
                errors.append(f'{axis}: {exc}')
        if errors:
            raise RuntimeError('servo off failed: ' + '; '.join(errors))
