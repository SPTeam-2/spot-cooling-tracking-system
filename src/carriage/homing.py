"""
[파일(내부 구현 코드) 목적]
왼쪽(모터 쪽) 리미트 스위치를 기준으로 캐리지 원점을 찾는 작동 알고리즘
[코드 내용]
HomingConfig: 속도·이격·단계별 최대 스텝·제한 시간·원점 이격 거리(mm) 검증
HomingProcedure: start() 후 update() 반복 / 단계별 진행·실패·취소 처리 / 성공 시 원점 확정 요청
HomingResult: 진행 상태, 현재 단계, 실패 이유
[플로우 간단 정리]
(왼쪽 눌림이면 RELEASE) → SEEK_FAST 빠른 접근 → BACKOFF 이격 → SEEK_SLOW 느린 재접근
→ OFFSET 원점 이격 → 해제 확인 → CarriageFunctions.set_home() → DONE

원점(0mm) = 2차 접촉점에서 +방향으로 home_offset_mm만큼 이격한 위치
update() 한 번에 최대 한 STEP / 단계마다 최대 스텝·시간 제한 / STOP 시 원점 미확인 유지
일반 이동 중 리미트 감지는 Homing 성공으로 취급하지 않음 (이 절차 안의 접근 단계만 접촉으로 인정)
참고: 개인 노션 #8-1(6장 Homing 플로우·구현 규칙), #11-1, #코드 구조 관련
팀 노션 4. 캐리지 이동 제어(원점측 스위치 기준 원점 설정) / coordinate.yaml rail.home_offset_mm
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
import time

from .function_stepper import StepperReason


class HomingStatus(str, Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    DONE = 'DONE'
    FAILED = 'FAILED'
    CANCELLED = 'CANCELLED'


class HomingPhase(str, Enum):
    NONE = 'NONE'
    RELEASE = 'RELEASE'        # 시작 시 왼쪽이 눌려 있으면 +방향으로 빠져나옴
    SEEK_FAST = 'SEEK_FAST'    # -방향 빠른 접근
    BACKOFF = 'BACKOFF'        # +방향 이격
    SEEK_SLOW = 'SEEK_SLOW'    # -방향 느린 재접근
    OFFSET = 'OFFSET'          # +방향 원점 이격
    SETTLE = 'SETTLE'          # 접촉 후 정지 상태에서 입력 재확인


@dataclass(frozen=True)
class HomingConfig:
    """실측 전 값은 기본값을 두지 않음. 호출자가 명시"""
    fast_speed_steps_s: float
    slow_speed_steps_s: float
    backoff_steps: int          # BACKOFF·RELEASE 이격량
    max_seek_steps: int         # SEEK_FAST 최대 스텝 (레일 전체 길이보다 크게)
    max_reseek_steps: int       # SEEK_SLOW 최대 스텝 (backoff_steps보다 크게)
    home_offset_mm: float       # 2차 접촉점 → 원점 거리
    phase_timeout_s: float      # 단계별 제한 시간
    settle_s: float = 0.0       # 접촉 감지 후 입력 재확인까지 대기

    def __post_init__(self):
        for name in ('fast_speed_steps_s', 'slow_speed_steps_s', 'home_offset_mm', 'phase_timeout_s'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if type(self.settle_s) not in (int, float) or not math.isfinite(self.settle_s) or self.settle_s < 0:
            raise ValueError('settle_s must be finite and non-negative')
        if self.settle_s >= self.phase_timeout_s:
            raise ValueError('settle_s must be shorter than phase_timeout_s')
        for name in ('backoff_steps', 'max_seek_steps', 'max_reseek_steps'):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f'{name} must be a positive integer')
        if self.slow_speed_steps_s > self.fast_speed_steps_s:
            raise ValueError('slow speed must not exceed fast speed')
        if self.max_reseek_steps <= self.backoff_steps:
            raise ValueError('max_reseek_steps must exceed backoff_steps')


@dataclass(frozen=True)
class HomingResult:
    status: HomingStatus = HomingStatus.IDLE
    phase: HomingPhase = HomingPhase.NONE
    error: str | None = None


class HomingProcedure:
    """CarriageFunctions를 전달받아 원점 탐색 단계를 진행. 일반 이동과 동시에 실행하지 않음"""

    def __init__(self, carriage, config, *, clock=time.monotonic):
        self._carriage = carriage
        self._config = config
        self._clock = clock
        self._offset_steps = carriage.geometry.mm_to_steps(config.home_offset_mm)
        if self._offset_steps <= 0:
            raise ValueError('home_offset_mm is smaller than one step')
        self._result = HomingResult()
        self._phase_started = 0.0
        self._settle_until = 0.0
        self._after_settle = HomingPhase.NONE

    @property
    def result(self):
        return self._result

    @property
    def offset_steps(self):
        return self._offset_steps

    def start(self):
        """원점을 무효화하고 탐색 시작. 이미 원점이 있어도 명시 요청이면 다시 찾음"""
        if self._result.status == HomingStatus.RUNNING:
            raise RuntimeError('homing already running')
        self._result = HomingResult(status=HomingStatus.RUNNING)
        self._carriage.invalidate('homing in progress')
        if self._carriage.result.reason == StepperReason.FAULT:
            return self._fail('motor in fault; reset fault first')
        if self._carriage.result.reason == StepperReason.RUNNING:
            return self._fail('stop current move first')
        try:
            state = self._carriage.read_limits()
        except Exception as exc:
            return self._fail(f'limit read failed: {exc}')
        if state.both_pressed:
            return self._fail('both limit switches pressed')
        if state.left_pressed:
            self._begin(HomingPhase.RELEASE, 1, self._config.backoff_steps, self._config.fast_speed_steps_s)
        else:
            # 오른쪽만 눌린 경우도 -방향 접근은 허용됨
            self._begin(HomingPhase.SEEK_FAST, -1, self._config.max_seek_steps, self._config.fast_speed_steps_s)
        return self._result

    def update(self, stop_requested=False):
        """STOP·제한 시간을 먼저 확인하고 현재 단계를 최대 한 STEP 진행"""
        if self._result.status != HomingStatus.RUNNING:
            return self._result
        if stop_requested:
            self._carriage.stop()
            self._carriage.invalidate('homing cancelled')
            self._result = replace(self._result, status=HomingStatus.CANCELLED)
            return self._result
        now = self._clock()
        if now - self._phase_started > self._config.phase_timeout_s:
            return self._fail(f'{self._result.phase.value} timeout')
        if self._result.phase == HomingPhase.SETTLE:
            if now >= self._settle_until:
                self._check_contact()
            return self._result
        result = self._carriage.update()
        if result.reason != StepperReason.RUNNING:
            self._on_move_end(result)
        return self._result

    def _begin(self, phase, direction, steps, speed):
        self._result = replace(self._result, phase=phase)
        self._phase_started = self._clock()
        result = self._carriage.start_move(direction, steps, speed_steps_s=speed)
        if result.reason != StepperReason.RUNNING:
            self._on_move_end(result)

    def _fail(self, message):
        try:
            self._carriage.stop()
        except Exception as exc:
            message += f'; stop failed: {exc}'
        self._carriage.invalidate(f'homing failed: {message}')
        self._result = replace(self._result, status=HomingStatus.FAILED, error=message)
        return self._result

    def _read(self):
        try:
            return self._carriage.read_limits()
        except Exception as exc:
            self._fail(f'limit read failed: {exc}')
            return None

    def _on_move_end(self, result):
        phase = self._result.phase
        if result.reason == StepperReason.FAULT:
            self._fail(f'motor fault: {result.error}')
        elif result.reason == StepperReason.RIGHT_LIMIT:
            self._fail('unexpected right limit')
        elif result.reason == StepperReason.STOPPED:
            self._fail('motor stopped outside homing control')
        elif phase in (HomingPhase.SEEK_FAST, HomingPhase.SEEK_SLOW):
            if result.reason == StepperReason.LEFT_LIMIT:
                # 접촉 감지 시 추가 펄스는 이미 차단됨. 정지 상태에서 입력 재확인
                self._after_settle = phase
                self._settle_until = self._clock() + self._config.settle_s
                self._result = replace(self._result, phase=HomingPhase.SETTLE)
                self._phase_started = self._clock()
            else:
                self._fail('left switch not found within step limit')
        elif result.reason == StepperReason.COMPLETED:
            self._after_backoff(phase)
        else:
            self._fail(f'unexpected result {result.reason.value} in {phase.value}')

    def _check_contact(self):
        state = self._read()
        if state is None:
            return
        if state.both_pressed:
            self._fail('both limit switches pressed')
        elif not state.left_pressed:
            self._fail('left contact not stable')
        elif self._after_settle == HomingPhase.SEEK_FAST:
            self._begin(HomingPhase.BACKOFF, 1, self._config.backoff_steps, self._config.fast_speed_steps_s)
        else:
            self._begin(HomingPhase.OFFSET, 1, self._offset_steps, self._config.slow_speed_steps_s)

    def _after_backoff(self, phase):
        state = self._read()
        if state is None:
            return
        if state.both_pressed or state.right_pressed:
            self._fail('unexpected limit state after moving away')
        elif state.left_pressed:
            self._fail(f'left switch not released after {phase.value}')
        elif phase == HomingPhase.RELEASE:
            self._begin(HomingPhase.SEEK_FAST, -1, self._config.max_seek_steps, self._config.fast_speed_steps_s)
        elif phase == HomingPhase.BACKOFF:
            self._begin(HomingPhase.SEEK_SLOW, -1, self._config.max_reseek_steps, self._config.slow_speed_steps_s)
        elif phase == HomingPhase.OFFSET:
            try:
                self._carriage.set_home()
            except Exception as exc:
                self._fail(f'set home failed: {exc}')
                return
            self._result = replace(self._result, status=HomingStatus.DONE)
        else:
            self._fail(f'unexpected completion in {phase.value}')
