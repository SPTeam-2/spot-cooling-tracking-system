"""
[파일(내부 구현 코드) 목적]
GT2 벨트·직결 풀리 사양을 검증하고 이동 거리(mm)와 스텝 수를 환산하는 계산 계층
[코드 내용]
DriveGeometry: 벨트 피치·풀리 잇수·기본 스텝·마이크로스텝 검증, steps_per_mm 계산
from_config()로 기존 설정 재사용 / mm_to_steps()·steps_to_mm()로 양방향 환산
[플로우 간단 정리]
기존 stepper 설정 전달 → 사양 검증 → 1mm당 스텝 계산 → 거리 또는 스텝 수 환산

모터 축에 풀리가 직접 연결된 구조 전제 / GPIO 접근 없음
환산 결과는 이론값이며 실제 위치 측정·탈조·벨트 미끄러짐 검출 기능은 아님
"""
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
import math


@dataclass(frozen=True)
class DriveGeometry:
    belt_pitch_mm: float
    pulley_teeth: int
    steps_per_rev: int
    microstep: int

    def __post_init__(self):
        if type(self.belt_pitch_mm) not in (int, float) or not math.isfinite(self.belt_pitch_mm) or self.belt_pitch_mm <= 0:
            raise ValueError('belt_pitch_mm must be finite and positive')
        for value in (self.pulley_teeth, self.steps_per_rev):
            if type(value) is not int or value <= 0:
                raise ValueError('teeth and steps_per_rev must be positive integers')
        if type(self.microstep) is not int or self.microstep not in (1, 2, 4, 8, 16):
            raise ValueError('unsupported A4988 microstep')
        if not math.isfinite(self.steps_per_mm) or self.steps_per_mm <= 0:
            raise ValueError('geometry outside numeric range')

    @classmethod
    def from_config(cls, config):
        """기존 coordinate.yaml의 stepper 설정 키를 재사용한다."""
        return cls(**{key: config['stepper'][key] for key in
                      ('belt_pitch_mm', 'pulley_teeth', 'steps_per_rev', 'microstep')})

    @property
    def steps_per_mm(self):
        """1mm당 이론 스텝 수: 기본 스텝 × 마이크로스텝 ÷ (벨트 피치 × 풀리 잇수)."""
        return self.steps_per_rev * self.microstep / (self.belt_pitch_mm * self.pulley_teeth)

    def mm_to_steps(self, distance_mm):
        """부호를 유지해 정수 스텝으로 환산한다. 정확히 반 스텝이면 절댓값을 올림한다."""
        if type(distance_mm) not in (int, float) or not math.isfinite(distance_mm):
            raise ValueError('distance must be finite')
        magnitude = Decimal(str(abs(distance_mm))) * Decimal(str(self.steps_per_mm))
        steps = int(magnitude.to_integral_value(rounding=ROUND_HALF_UP))
        return -steps if distance_mm < 0 else steps

    def steps_to_mm(self, steps):
        """출력 스텝 수를 이론 거리(mm)로 환산한다. 탈조·벨트 미끄러짐은 반영하지 않는다."""
        if type(steps) is not int:
            raise ValueError('steps must be an integer')
        return steps / self.steps_per_mm
