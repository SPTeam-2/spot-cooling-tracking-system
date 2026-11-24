"""
[파일(내부 구현 코드) 목적]
원점이 확정된 캐리지를 목표 위치(mm)로 이동시키는 작동 알고리즘
[코드 내용]
TargetMoveConfig: 이동 속도·허용 범위(mm) 검증
TargetMover: set_target()으로 목표 접수 / update() 반복으로 이동·도착·실패 처리 / stop()으로 취소
TargetResult: 진행 상태, 목표 mm, 현재 추정 위치 mm, 실패 이유
[플로우 간단 정리]
set_target() → 원점·값·범위 검사 → (목표 − 현재)를 부호 있는 스텝으로 환산 → 이동 접수
→ update() 반복 → 도착(ARRIVED) / 취소(CANCELLED) / 실패(FAILED)
이동 중 새 목표 → 현재 이동 정지 → 그 자리에서 새 목표로 바로 이동 (원점 경유 없음)

목표 0mm = Homing 원점 (왼쪽 스위치 2차 접촉점 + home_offset_mm)
범위 밖·NaN·원점 미확정 목표는 거절 (끝값으로 자르기는 통합 제어 쪽에서 결정)
목표 이동 중 리미트 접촉 = 위치 추정이 틀린 것으로 보고 FAILED + 원점 무효 (다시 Homing 필요)
정상 취소(STOP)는 위치 손실이 아니므로 원점 유지
update() 한 번에 최대 한 STEP / 일정 속도 이동 (가감속은 #11-3) / 작은 흔들림 무시(deadzone)는 통합 제어 쪽
참고: 개인 노션 #11-2, #코드 구조 관련 / coordinate.yaml rail.travel_mm
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .function_stepper import StepperReason


class TargetStatus(str, Enum):
    IDLE = 'IDLE'
    MOVING = 'MOVING'
    ARRIVED = 'ARRIVED'
    CANCELLED = 'CANCELLED'
    FAILED = 'FAILED'


def _finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class TargetMoveConfig:
    """실측 전 값은 기본값을 두지 않음. 호출자가 명시"""
    speed_steps_s: float
    min_mm: float          # 허용 범위 시작 (원점 기준)
    max_mm: float          # 허용 범위 끝 (원점 기준)

    def __post_init__(self):
        if not _finite_number(self.speed_steps_s) or self.speed_steps_s <= 0:
            raise ValueError('speed_steps_s must be finite and positive')
        if not _finite_number(self.min_mm) or not _finite_number(self.max_mm):
            raise ValueError('range must be finite')
        if self.min_mm < 0:
            raise ValueError('min_mm must not be negative (0mm is the home position)')
        if self.max_mm <= self.min_mm:
            raise ValueError('max_mm must exceed min_mm')


@dataclass(frozen=True)
class TargetResult:
    status: TargetStatus = TargetStatus.IDLE
    target_mm: float | None = None
    position_mm: float | None = None
    error: str | None = None


class TargetMover:
    """CarriageFunctions를 전달받아 목표 위치로 이동. Homing과 동시에 실행하지 않음. 단일 스레드에서 호출"""

    def __init__(self, carriage, config):
        self._carriage = carriage
        self._config = config
        self._status = TargetStatus.IDLE
        self._target_mm = None
        self._target_steps = None
        self._error = None

    @property
    def result(self):
        return TargetResult(self._status, self._target_mm, self._carriage.position_mm, self._error)

    def set_target(self, target_mm):
        """목표 접수. 거절 시 예외를 내고 진행 중인 이동은 그대로 유지"""
        if not _finite_number(target_mm):
            raise ValueError('target_mm must be a finite number')
        if not self._carriage.homed:
            raise RuntimeError(f'not homed: {self._carriage.invalid_reason}')
        if not self._config.min_mm <= target_mm <= self._config.max_mm:
            raise ValueError(f'target {target_mm}mm outside range '
                             f'{self._config.min_mm}~{self._config.max_mm}mm')
        target_steps = self._carriage.geometry.mm_to_steps(target_mm)
        if self._status == TargetStatus.MOVING and target_steps == self._target_steps:
            self._target_mm = target_mm   # 같은 스텝 목표면 이동을 끊지 않음
            return self.result
        if self._carriage.result.reason == StepperReason.RUNNING:
            self._carriage.stop()         # 실제 출력한 스텝까지만 위치에 반영된 상태에서 정지
        self._target_mm, self._target_steps, self._error = target_mm, target_steps, None
        delta = target_steps - self._carriage.position_steps
        if delta == 0:
            self._status = TargetStatus.ARRIVED
            return self.result
        self._status = TargetStatus.MOVING
        try:
            moved = self._carriage.start_move(1 if delta > 0 else -1, abs(delta),
                                              speed_steps_s=self._config.speed_steps_s)
        except Exception as exc:
            # 속도 상한 초과 등 이동 접수 전 거절. 펄스가 없으므로 원점은 유지
            self._status, self._error = TargetStatus.FAILED, f'move rejected: {exc}'
            raise
        if moved.reason != StepperReason.RUNNING:
            self._on_move_end(moved)
        return self.result

    def update(self, stop_requested=False):
        """STOP을 먼저 확인하고 최대 한 STEP 진행"""
        if self._status != TargetStatus.MOVING:
            return self.result
        if stop_requested:
            return self.stop()
        moved = self._carriage.update()
        if moved.reason != StepperReason.RUNNING:
            self._on_move_end(moved)
        return self.result

    def stop(self):
        """진행 중인 목표 이동 취소. 원점은 유지"""
        if self._status == TargetStatus.MOVING:
            self._carriage.stop()
            self._status = TargetStatus.CANCELLED
        return self.result

    def _on_move_end(self, moved):
        if moved.reason == StepperReason.COMPLETED:
            if self._carriage.position_steps == self._target_steps:
                self._status = TargetStatus.ARRIVED
            else:
                self._fail('position mismatch after move')
        elif moved.reason in (StepperReason.LEFT_LIMIT, StepperReason.RIGHT_LIMIT):
            self._fail(f'unexpected {moved.reason.value.lower()} during target move; homing required')
        elif moved.reason == StepperReason.FAULT:
            self._fail(f'motor fault: {moved.error}')   # 원점 무효는 CarriageFunctions가 처리
        elif moved.reason == StepperReason.STOPPED:
            self._status = TargetStatus.CANCELLED
        else:
            self._fail(f'unexpected result {moved.reason.value}')

    def _fail(self, message):
        self._carriage.invalidate(f'target move failed: {message}')
        self._status = TargetStatus.FAILED
        self._error = message
