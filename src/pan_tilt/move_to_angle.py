"""
[파일(내부 구현 코드) 목적]
현재 명령 각도에서 목표 각도(pan·tilt)까지 최대 각속도로 조금씩 이동시키는 작동 알고리즘
[코드 내용]
MoveToAngleConfig: 최대 각속도(deg/s)·펄스 갱신 간격 검증
MoveToAngle: set_target()으로 목표 접수 / update() 반복으로 두 축을 함께 이동 / stop()으로 취소
AngleResult: 진행 상태, 목표 각도, 현재 명령 각도, 실패 이유
[플로우 간단 정리]
set_target() → 준비 상태·범위 검사 → update()마다 경과 시간 × 최대 각속도만큼 목표에 다가감
→ PanTiltFunctions.set_angle() → 두 축 도착 시 ARRIVED / STOP 시 CANCELLED / 출력 오류 시 FAILED

갱신 간격(기본 0.02s)보다 자주 호출해도 펄스는 그 간격마다만 바꿈
호출이 늦어져도 한 번에 움직이는 양은 갱신 간격 5배 시간 분량까지로 제한 (갑작스러운 큰 회전 방지)
이동 중 새 목표 → 현재 명령 각도에서 바로 새 목표로 / STOP·도착 후에는 마지막 펄스를 유지 (힘 유지)
출력 오류 시 두 축 출력 끄기 시도 / 실제 각도 측정 없음 (명령 각도 기준)
참고: 개인 노션 #12(목표 각도 알고리즘), #코드 구조 관련(1장 move_to_angle.py)
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time

from .function_pan_tilt import AXES


class AngleStatus(str, Enum):
    IDLE = 'IDLE'
    MOVING = 'MOVING'
    ARRIVED = 'ARRIVED'
    CANCELLED = 'CANCELLED'
    FAILED = 'FAILED'


@dataclass(frozen=True)
class MoveToAngleConfig:
    max_speed_deg_s: float          # 축별 최대 각속도, 실측 전 값은 호출자가 명시
    update_interval_s: float = 0.02  # 펄스 갱신 간격 (서보 신호 주기 20ms 기준)

    def __post_init__(self):
        for name in ('max_speed_deg_s', 'update_interval_s'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')


@dataclass(frozen=True)
class AngleResult:
    status: AngleStatus = AngleStatus.IDLE
    target_pan: float | None = None
    target_tilt: float | None = None
    pan: float | None = None
    tilt: float | None = None
    error: str | None = None


class MoveToAngle:
    """PanTiltFunctions를 전달받아 목표 각도로 이동. 단일 스레드에서 호출"""

    def __init__(self, pan_tilt, config, *, clock=time.monotonic):
        self._pan_tilt = pan_tilt
        self._config = config
        self._clock = clock
        self._status = AngleStatus.IDLE
        self._target = {'pan': None, 'tilt': None}
        self._last = 0.0
        self._error = None

    @property
    def result(self):
        return AngleResult(self._status, self._target['pan'], self._target['tilt'],
                           self._pan_tilt.angle('pan'), self._pan_tilt.angle('tilt'), self._error)

    def set_target(self, pan_deg, tilt_deg):
        """목표 접수. 거절 시 예외를 내고 진행 중인 이동은 그대로 유지"""
        if not self._pan_tilt.ready:
            raise RuntimeError('commanded angle unknown; call center() first')
        self._pan_tilt.check_angle('pan', pan_deg)
        self._pan_tilt.check_angle('tilt', tilt_deg)
        if self._status != AngleStatus.MOVING:
            self._last = self._clock()
        self._target = {'pan': pan_deg, 'tilt': tilt_deg}
        self._error = None
        self._status = AngleStatus.ARRIVED if self._arrived() else AngleStatus.MOVING
        return self.result

    def _arrived(self):
        return all(self._pan_tilt.angle(axis) == self._target[axis] for axis in AXES)

    def update(self, stop_requested=False):
        """갱신 간격이 지났으면 두 축을 최대 각속도만큼 목표에 다가가게 함"""
        if self._status != AngleStatus.MOVING:
            return self.result
        if stop_requested:
            return self.stop()
        now = self._clock()
        elapsed = now - self._last
        if elapsed + 1e-9 < self._config.update_interval_s:   # 부동소수점 오차로 정주기 호출을 건너뛰지 않게
            return self.result
        self._last = now
        max_step = self._config.max_speed_deg_s * min(elapsed, 5 * self._config.update_interval_s)
        try:
            for axis in AXES:
                current, target = self._pan_tilt.angle(axis), self._target[axis]
                diff = target - current
                if diff == 0:
                    continue
                new = target if abs(diff) <= max_step else current + math.copysign(max_step, diff)
                self._pan_tilt.set_angle(axis, new)
        except Exception as exc:
            return self._fail(f'servo output failed: {exc}')
        if self._arrived():
            self._status = AngleStatus.ARRIVED
        return self.result

    def stop(self):
        """진행 중인 이동 취소. 마지막 펄스는 유지"""
        if self._status == AngleStatus.MOVING:
            self._status = AngleStatus.CANCELLED
        return self.result

    def _fail(self, message):
        try:
            self._pan_tilt.off()
        except Exception as exc:
            message += f'; off failed: {exc}'
        self._status = AngleStatus.FAILED
        self._error = message
        return self.result
