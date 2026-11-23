"""
[파일(내부 구현 코드) 목적]
모터 이동 요청을 관리하고 리미트·STOP·오류에 따라 추가 STEP을 차단하는 기능 계층
[코드 내용]
StepperFunctions: 방향·스텝·속도 검증, 유한 스텝 이동, 방향별 리미트 차단, 정지·오류 잠금
StepperReason·StepperResult: 진행 상태, 완료 펄스 수, 중단 이유, 위치 불확실 여부 제공
[플로우 간단 정리]
start_move()로 이동 접수 → update()에서 STOP·리미트 확인 → 최대 한 STEP 출력 → 완료 또는 중단

한 스레드의 실행 루프에서 호출 / 반대 방향은 새 요청으로 후퇴 / 자동 재개 없음
완료 펄스 수는 실제 위치 측정값이 아님 / 절대 위치 제어·Homing은 포함하지 않음
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
import time


class StepperReason(str, Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    COMPLETED = 'COMPLETED'
    STOPPED = 'STOPPED'
    LEFT_LIMIT = 'LEFT_LIMIT'
    RIGHT_LIMIT = 'RIGHT_LIMIT'
    FAULT = 'FAULT'


@dataclass(frozen=True)
class StepperResult:
    """이동 결과//completed_steps는 확인된 출력 수이며 실제 위치 측정값이 아니다."""
    reason: StepperReason = StepperReason.IDLE
    direction: int = 0
    requested_steps: int = 0
    completed_steps: int = 0
    error: str | None = None
    position_uncertain: bool = False


class StepperFunctions:
    """리미트·STOP·오류를 확인한 뒤 HW에 한 STEP씩 요청한다. 단일 스레드에서 호출한다."""
    def __init__(self, hw, limits, *, max_speed_steps_s, clock=time.monotonic):
        self._positive(max_speed_steps_s)
        self._hw, self._limits, self._clock = hw, limits, clock
        self.max_speed_steps_s = max_speed_steps_s
        self._result = StepperResult()
        self._last_time = float('-inf')
        self._next = 0.0
        self._interval = 0.0

    @staticmethod
    def _positive(value):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('speed must be finite and positive')

    @property
    def result(self):
        return self._result

    def _now(self):
        value = self._clock()
        if not math.isfinite(value) or value < self._last_time:
            raise ValueError('clock must be finite and monotonic')
        self._last_time = value
        return value

    def _fault(self, exc, uncertain=False):
        message = str(exc)
        try:
            self._hw.disable()
        except Exception as stop_exc:
            message += f'; disable failed: {stop_exc}'
        self._result = replace(self._result, reason=StepperReason.FAULT,
                               error=message, position_uncertain=uncertain or self._result.position_uncertain)

    def _blocked(self, direction):
        state = self._limits.read_state()
        if state.both_pressed:
            raise RuntimeError('both limit switches pressed')
        if direction == -1 and state.left_pressed:
            return StepperReason.LEFT_LIMIT
        if direction == 1 and state.right_pressed:
            return StepperReason.RIGHT_LIMIT
        return None

    def start_move(self, direction, steps, *, speed_steps_s):
        """방향·스텝·속도를 검사하고 이동을 접수한다. 실제 STEP은 update에서 출력한다."""
        if self.result.reason in (StepperReason.RUNNING, StepperReason.FAULT):
            raise RuntimeError('stop current move or reset fault first')
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('direction must be -1 or +1')
        if type(steps) is not int or steps <= 0:
            raise ValueError('steps must be a positive integer; skip zero-step conversions')
        self._positive(speed_steps_s)
        if speed_steps_s > self.max_speed_steps_s:
            raise ValueError('speed exceeds configured limit')
        interval = 1.0 / speed_steps_s
        if not math.isfinite(interval):
            raise ValueError('speed too small')
        self._result = StepperResult(direction=direction, requested_steps=steps)
        try:
            self._hw.check_connection()
            blocked = self._blocked(direction)
            if blocked:
                self._result = replace(self.result, reason=blocked)
                return self.result
            self._hw.set_direction(direction)
            self._hw.enable()
            self._next = self._now()
            self._interval = interval
            self._result = replace(self.result, reason=StepperReason.RUNNING)
        except Exception as exc:
            self._fault(exc)
        return self.result

    def update(self, stop_requested=False):
        """STOP·리미트를 우선 확인하고 출력 시각이 됐을 때 최대 한 STEP 진행한다."""
        if self.result.reason != StepperReason.RUNNING:
            return self.result
        if stop_requested:
            return self.stop()
        pulse_started = False
        try:
            self._hw.check_connection()
            blocked = self._blocked(self.result.direction)
            if blocked:
                self._result = replace(self.result, reason=blocked)
                return self.result
            now = self._now()
            if now < self._next:
                return self.result
            pulse_started = True
            self._hw.step_once()
            self._result = replace(self.result, completed_steps=self.result.completed_steps + 1)
            pulse_started = False
            # 펄스 완료 시각부터 다음 간격을 계산한다. 호출이 늦어져도 밀린 펄스를 몰아서 내보내지 않는다.
            self._next = self._now() + self._interval
            if self.result.completed_steps == self.result.requested_steps:
                self._result = replace(self.result, reason=StepperReason.COMPLETED)
        except Exception as exc:
            self._fault(exc, uncertain=pulse_started)
        return self.result

    def stop(self):
        """현재 이동을 취소한다. 잔여 스텝을 자동 재개하지 않으며 정상 정지 토크는 유지한다."""
        if self.result.reason == StepperReason.RUNNING:
            self._result = replace(self.result, reason=StepperReason.STOPPED)
        return self.result

    def reset_fault(self):
        """연결·리미트·비활성화 성공을 확인해 오류 잠금을 해제한다. 실제 위치는 복구하지 않는다."""
        if self.result.reason != StepperReason.FAULT:
            raise RuntimeError('not in fault')
        try:
            self._hw.check_connection()
            if self._limits.read_state().both_pressed:
                raise RuntimeError('both limit switches still pressed')
            self._hw.disable()
        except Exception as exc:
            self._fault(exc)
            return self.result
        self._result = StepperResult()
        return self.result
