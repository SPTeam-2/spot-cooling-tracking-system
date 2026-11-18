"""
[파일(내부 구현 코드) 목적]
캐리지의 원점 유효 여부와 현재 추정 위치를 한 곳에서 관리하는 기능 계층
[코드 내용]
CarriageFunctions: StepperFunctions 이동 요청 전달, 실제 출력한 스텝만 위치에 누적
set_home()으로 원점 확정, invalidate()로 원점 무효화, position_steps·position_mm 제공
[플로우 간단 정리]
start_move() → update() 반복 → 새로 출력된 스텝 수 확인 → 원점 유효 시 위치 갱신 → 오류·위치 불확실 시 원점 무효

위치 단위: 내부 스텝, 외부 mm / 원점 미확인 시 위치는 None (0mm와 구분)
방향: -1 모터 쪽(왼쪽), +1 모터에서 멀어지는 쪽 / 원점은 왼쪽 접촉점에서 이격한 위치
목표 mm 이동·0~L 범위 검사·도착 판정은 #11-2 후속 작업
참고: 개인 노션 #코드 구조 관련(3장 계층별 책임), #8-1(4장 현재 위치, Q6 위치 기준 유효성)
"""
from .function_stepper import StepperReason


class CarriageFunctions:
    """위치·원점 상태의 단일 관리 창구. 단일 스레드에서 호출"""

    def __init__(self, motor, limits, geometry):
        self._motor = motor
        self._limits = limits
        self.geometry = geometry
        self._homed = False
        self._position_steps = None
        self._counted = 0
        self.invalid_reason = 'not homed yet'

    @property
    def homed(self):
        return self._homed

    @property
    def position_steps(self):
        """원점 기준 추정 위치(스텝). 원점 미확인 시 None"""
        return self._position_steps if self._homed else None

    @property
    def position_mm(self):
        """원점 기준 추정 위치(mm). 탈조·벨트 미끄러짐은 반영하지 못함"""
        if not self._homed:
            return None
        return self.geometry.steps_to_mm(self._position_steps)

    @property
    def result(self):
        return self._motor.result

    def read_limits(self):
        """좌우 스위치 상태를 즉시 읽음. 입력 오류는 호출자에게 전달"""
        return self._limits.read_state()

    def start_move(self, direction, steps, *, speed_steps_s):
        """유한 스텝 이동 접수. 실제 출력은 update()에서 진행"""
        self._counted = 0
        result = self._motor.start_move(direction, steps, speed_steps_s=speed_steps_s)
        self._after(result)
        return result

    def update(self, stop_requested=False):
        """모터를 최대 한 STEP 진행하고 새로 출력된 스텝만 위치에 반영"""
        result = self._motor.update(stop_requested)
        self._after(result)
        return result

    def stop(self):
        """추가 펄스 중단. 원점은 유지 (정상 정지는 위치 손실이 아님)"""
        result = self._motor.stop()
        self._after(result)
        return result

    def _after(self, result):
        new_steps = result.completed_steps - self._counted
        if new_steps > 0 and self._homed:
            self._position_steps += result.direction * new_steps
        self._counted = result.completed_steps
        if result.reason == StepperReason.FAULT or result.position_uncertain:
            self.invalidate('motor fault or uncertain pulse')

    def set_home(self):
        """현재 위치를 원점 0으로 확정. Homing 알고리즘이 최종 이격 성공 후에만 호출"""
        if self._motor.result.reason in (StepperReason.RUNNING, StepperReason.FAULT):
            raise RuntimeError('cannot set home while moving or in fault')
        self._homed = True
        self._position_steps = 0
        self.invalid_reason = None

    def invalidate(self, reason):
        """원점 무효화. 다시 Homing하기 전까지 위치를 믿지 않음"""
        self._homed = False
        self._position_steps = None
        self.invalid_reason = reason

    def reset_fault(self):
        """모터 오류 잠금 해제. 위치는 복구하지 않으므로 원점은 무효 유지"""
        result = self._motor.reset_fault()
        self.invalidate('fault reset; homing required')
        return result
