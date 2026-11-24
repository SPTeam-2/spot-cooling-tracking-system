"""
[파일(내부 구현 코드) 목적]
모터 이동 요청을 관리하고 리미트·STOP·오류에 따라 추가 STEP을 차단하는 기능 계층
[코드 내용]
StepperFunctions: 방향·스텝·속도 검증, 유한 스텝 이동, 방향별 리미트 차단, 정지·오류 잠금
StepperReason·StepperResult: 진행 상태, 완료 펄스 수, 중단 이유, 위치 불확실 여부 제공
build_pulse_plan(): 160Hz 출발 단계 가속·감속 계획 (가감속 펄스도 요청 스텝에 포함)
[플로우 간단 정리]
start_move()로 이동 접수 → update()에서 STOP·리미트 확인 → 펄스 진행 반영 → 완료 또는 중단

펄스 방식 (pulse_mode)
- 'pwm' (방식 A): HW가 PWM으로 펄스를 내고 콜백으로 센다. update()는 카운트 반영·리미트·STOP·시간 초과만 확인
  속도는 pigpio PWM 허용 주파수만 가능 (현장 안정값 200·250Hz), 160Hz부터 단계 가감속
- 'step' (방식 B, 예비): update() 한 번에 최대 한 STEP 직접 출력 (기존 방식)
  기존 모의 테스트·모의 데모의 가짜 모터(step_once만 있음)는 수정 없이 이 방식으로 동작
- 'auto' (기본): HW에 start_pulses()가 있으면 'pwm'(실제 모터), 없으면 'step'(기존 가짜 모터)

한 스레드의 실행 루프에서 호출 / 반대 방향은 새 요청으로 후퇴 / 자동 재개 없음
완료 펄스 수는 Pi가 낸 펄스 수이며 실제 위치 측정값이 아님 / 절대 위치 제어·Homing은 포함하지 않음
방식 A는 정지 명령 뒤 늦게 센 펄스(초과 스텝)까지 completed_steps에 포함 → 위치 계산은 그대로 정확
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
import time

# pigpio 기본 설정(샘플링 5µs)에서 set_PWM_frequency로 정확히 설정되는 주파수
PWM_ALLOWED_HZ = (10, 20, 40, 50, 80, 100, 160, 200, 250, 320, 400, 500, 800, 1000, 1600, 2000)
RAMP_START_HZ = 160      # 현장 점검(11-22~23): 160Hz 출발 단계 가속에서 안정
RAMP_STAGE_STEPS = 40    # 가감속 단계 하나당 스텝 (1/8 기준 1mm)


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


def build_pulse_plan(total, top_hz, *, ramp_start_hz=RAMP_START_HZ, stage_steps=RAMP_STAGE_STEPS,
                     allowed_hz=PWM_ALLOWED_HZ):
    """[(이 카운트부터, 주파수Hz), ...] 계획. 가속 → 정속 → 감속, 가감속 펄스도 total에 포함

    가속 단계 = ramp_start_hz 이상 top_hz 미만의 허용 주파수.
    짧은 이동이라 가감속 자리가 모자라면 가장 높은 단계(정속 포함)부터 뺀다 → 짧은 이동은 낮은 속도로만 이동.
    """
    levels = [f for f in allowed_hz if ramp_start_hz <= f < top_hz] + [top_hz]
    while len(levels) > 1 and (len(levels) - 1) * 2 * stage_steps >= total:
        levels.pop()
    ramp, top = levels[:-1], levels[-1]
    plan, count = [], 0
    for hz in ramp:
        plan.append((count, hz))
        count += stage_steps
    plan.append((count, top))
    decel_start = total - len(ramp) * stage_steps
    for i, hz in enumerate(reversed(ramp)):
        plan.append((decel_start + i * stage_steps, hz))
    return plan


def plan_duration_s(plan, total):
    """계획대로 출력할 때 걸리는 이론 시간(초)"""
    edges = [at for at, _ in plan[1:]] + [total]
    return sum((end - at) / hz for (at, hz), end in zip(plan, edges))


class StepperFunctions:
    """리미트·STOP·오류를 확인하며 HW 펄스 출력을 관리한다. 단일 스레드에서 호출한다."""
    def __init__(self, hw, limits, *, max_speed_steps_s, clock=time.monotonic, pulse_mode='auto',
                 ramp_start_hz=RAMP_START_HZ, ramp_stage_steps=RAMP_STAGE_STEPS, timeout_margin_s=3.0):
        self._positive(max_speed_steps_s)
        if pulse_mode not in ('auto', 'pwm', 'step'):
            raise ValueError("pulse_mode must be 'auto', 'pwm' or 'step'")
        if pulse_mode == 'auto':
            pulse_mode = 'pwm' if callable(getattr(type(hw), 'start_pulses', None)) else 'step'
        if type(ramp_stage_steps) is not int or ramp_stage_steps <= 0:
            raise ValueError('ramp_stage_steps must be a positive integer')
        self._positive(ramp_start_hz)
        self._positive(timeout_margin_s)
        self._hw, self._limits, self._clock = hw, limits, clock
        self.max_speed_steps_s = max_speed_steps_s
        self.pulse_mode = pulse_mode
        self._ramp_start, self._stage = ramp_start_hz, ramp_stage_steps
        self._margin = timeout_margin_s
        self._result = StepperResult()
        self._last_time = float('-inf')
        self._next = 0.0
        self._interval = 0.0
        self._deadline = 0.0
        self._pulsing = False

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
            self._hw.disable()   # 방식 A에서는 PWM 정지·콜백 해제도 함께
        except Exception as stop_exc:
            message += f'; disable failed: {stop_exc}'
        self._pulsing = False
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
        """방향·스텝·속도를 검사하고 이동을 접수한다. 방식 A는 여기서 펄스 출력을 시작한다."""
        if self.result.reason in (StepperReason.RUNNING, StepperReason.FAULT):
            raise RuntimeError('stop current move or reset fault first')
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('direction must be -1 or +1')
        if type(steps) is not int or steps <= 0:
            raise ValueError('steps must be a positive integer; skip zero-step conversions')
        self._positive(speed_steps_s)
        if speed_steps_s > self.max_speed_steps_s:
            raise ValueError('speed exceeds configured limit')
        if self.pulse_mode == 'pwm' and speed_steps_s not in PWM_ALLOWED_HZ:
            raise ValueError(f'PWM speed must be one of {PWM_ALLOWED_HZ} steps/s')
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
            now = self._now()
            if self.pulse_mode == 'pwm':
                plan = build_pulse_plan(steps, int(speed_steps_s), ramp_start_hz=self._ramp_start,
                                        stage_steps=self._stage)
                self._deadline = now + plan_duration_s(plan, steps) + self._margin
                self._pulsing = True
                self._hw.start_pulses(steps, plan)
            else:
                self._next = now
                self._interval = interval
            self._result = replace(self.result, reason=StepperReason.RUNNING)
        except Exception as exc:
            self._fault(exc, uncertain=self._pulsing)
        return self.result

    def update(self, stop_requested=False):
        """STOP·리미트를 우선 확인하고 펄스 진행을 반영한다 (방식 B는 최대 한 STEP 출력)."""
        if self.result.reason != StepperReason.RUNNING:
            return self.result
        if stop_requested:
            return self.stop()
        if self.pulse_mode == 'pwm':
            return self._update_pwm()
        return self._update_step()

    def _update_pwm(self):
        try:
            self._hw.check_connection()
            now = self._now()
            status = self._hw.pulse_status()
            if status.error is not None:
                raise RuntimeError(f'pulse callback error: {status.error}')
            self._result = replace(self.result, completed_steps=status.count)
            blocked = self._blocked(self.result.direction)
            if blocked:
                return self._halt(blocked)
            if status.reached:
                return self._halt(StepperReason.COMPLETED)
            if now > self._deadline:
                raise TimeoutError('pulse count did not reach target in time')   # _fault에서 PWM 정지
        except Exception as exc:
            self._fault(exc, uncertain=True)
        return self.result

    def _halt(self, reason):
        """방식 A: PWM 정지 → 늦은 알림까지 집계한 최종 카운트로 결과 확정"""
        count = self._hw.finish_pulses()
        self._pulsing = False
        self._result = replace(self.result, reason=reason, completed_steps=count)
        return self.result

    def _update_step(self):
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
        if self.result.reason != StepperReason.RUNNING:
            return self.result
        if self.pulse_mode == 'pwm':
            try:
                return self._halt(StepperReason.STOPPED)
            except Exception as exc:
                self._fault(exc, uncertain=True)
                return self.result
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
        self._pulsing = False
        self._result = StepperResult()
        return self.result
