"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 가상 레일로 Homing 절차와 캐리지 위치 관리를 자동 검증
[코드 내용]
VirtualRail: 모터 위치와 좌우 스위치 입력이 같은 위치값을 공유하는 가상 장치 (해제 히스테리시스·고장 재현)
HomingTests: 정상 원점, 시작 위치별 재현성, 실패·취소, 원점 이후 위치 누적 검사
[플로우 간단 정리]
가상 레일·시계 준비 → Homing start() → update() 반복 → 상태·원점 물리 위치·homed 확인

알고리즘 확인용 PC 검사이며 실제 스위치 위치·이격 거리·정지거리 검증을 대체하지 않음
"""
import unittest
from src.carriage.drive_geometry import DriveGeometry
from src.carriage.function_carriage import CarriageFunctions
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions, StepperReason as R
from src.carriage.homing import HomingConfig, HomingProcedure, HomingStatus as S


class VirtualRail:
    """위치 단위는 스텝. left_at 이하에서 왼쪽 눌림, right_at 이상에서 오른쪽 눌림"""

    def __init__(self, position, *, left_at=0, right_at=5000, release_extra=0,
                 left_stuck=None, step_error_at=None):
        self.position = position
        self.direction = 1
        self.left_at, self.right_at = left_at, right_at
        self.release_extra = release_extra   # 눌린 뒤 이만큼 더 벗어나야 해제
        self.left_stuck = left_stuck         # True: 항상 눌림, False: 항상 안 눌림
        self.step_error_at = step_error_at
        self._left_latched = position <= left_at
        self.steps = 0

    def _left_pressed(self):
        if self.left_stuck is not None:
            return self.left_stuck
        if self.position <= self.left_at:
            self._left_latched = True
        elif self.position > self.left_at + self.release_extra:
            self._left_latched = False
        return self._left_latched

    def read_levels(self):
        # 눌림=LOW(0)
        return int(not self._left_pressed()), int(not self.position >= self.right_at)

    def check_connection(self): pass
    def set_direction(self, direction): self.direction = direction
    def enable(self): pass
    def disable(self): pass

    def step_once(self):
        if self.step_error_at is not None and self.steps == self.step_error_at:
            raise OSError('virtual pulse error')
        self.position += self.direction
        self.steps += 1


def config(**overrides):
    values = dict(fast_speed_steps_s=100, slow_speed_steps_s=50, backoff_steps=160,
                  max_seek_steps=6000, max_reseek_steps=400, home_offset_mm=5,
                  phase_timeout_s=1000, settle_s=0.02)
    values.update(overrides)
    return HomingConfig(**values)


class HomingTests(unittest.TestCase):
    def build(self, rail, **overrides):
        self.rail = rail
        self.now = 0.0
        clock = lambda: self.now
        limits = LimitSwitchFunctions(rail)
        motor = StepperFunctions(rail, limits, max_speed_steps_s=100, clock=clock)
        self.carriage = CarriageFunctions(motor, limits, DriveGeometry(2, 20, 200, 16))
        self.homing = HomingProcedure(self.carriage, config(**overrides), clock=clock)

    def run_homing(self, limit=200000, stop_at_step=None, started=False):
        result = self.homing.result if started else self.homing.start()
        for _ in range(limit):
            if result.status != S.RUNNING:
                return result
            self.now += 0.011
            stop = stop_at_step is not None and self.rail.steps >= stop_at_step
            result = self.homing.update(stop_requested=stop)
        self.fail('homing loop bound exceeded')

    def move(self, direction, steps):
        result = self.carriage.start_move(direction, steps, speed_steps_s=100)
        for _ in range(steps * 3 + 10):
            if result.reason != R.RUNNING:
                return result
            self.now += 0.011
            result = self.carriage.update()
        self.fail('move loop bound exceeded')

    def test_normal_homing_sets_origin_after_offset(self):
        self.build(VirtualRail(2000))
        self.assertIsNone(self.carriage.position_mm)
        result = self.run_homing()
        self.assertEqual(result.status, S.DONE, result.error)
        self.assertEqual(self.homing.offset_steps, 400)   # 5mm × 80스텝/mm
        self.assertEqual(self.rail.position, 400)          # 2차 접촉점(0) + 400
        self.assertTrue(self.carriage.homed)
        self.assertEqual(self.carriage.position_steps, 0)
        self.assertEqual(self.carriage.position_mm, 0.0)

    def test_same_origin_from_different_starts(self):
        for start in (2000, 37, 0, -3, 4999, 5000):
            with self.subTest(start=start):
                self.build(VirtualRail(start))
                result = self.run_homing()
                self.assertEqual(result.status, S.DONE, result.error)
                self.assertEqual(self.rail.position, 400)

    def test_left_pressed_at_start_releases_first(self):
        self.build(VirtualRail(-10))
        self.homing.start()
        self.assertEqual(self.homing.result.phase.value, 'RELEASE')
        self.assertEqual(self.run_homing(started=True).status, S.DONE)
        self.assertEqual(self.rail.position, 400)

    def test_both_pressed_at_start_fails(self):
        self.build(VirtualRail(0, right_at=0))
        result = self.run_homing()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('both', result.error)
        self.assertEqual(self.rail.steps, 0)
        self.assertFalse(self.carriage.homed)

    def test_switch_never_found_fails_within_step_limit(self):
        self.build(VirtualRail(2000, left_stuck=False))
        result = self.run_homing()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('not found', result.error)
        self.assertEqual(self.rail.steps, 6000)

    def test_switch_stuck_pressed_fails(self):
        self.build(VirtualRail(2000, left_stuck=True))
        result = self.run_homing()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('not released', result.error)
        self.assertFalse(self.carriage.homed)

    def test_offset_shorter_than_release_distance_fails(self):
        # 스위치가 5mm(400스텝)보다 더 떨어져야 해제되면 원점 확정 불가 → 이격 거리 재논의 필요
        self.build(VirtualRail(2000, release_extra=500), backoff_steps=600, max_reseek_steps=800)
        result = self.run_homing()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('OFFSET', result.error)
        self.assertFalse(self.carriage.homed)

    def test_right_limit_during_offset_fails(self):
        self.build(VirtualRail(2000, right_at=300))
        result = self.run_homing()
        self.assertEqual(result.status, S.FAILED)
        self.assertFalse(self.carriage.homed)

    def test_stop_cancels_without_origin_or_extra_pulse(self):
        self.build(VirtualRail(2000))
        result = self.run_homing(stop_at_step=100)
        self.assertEqual(result.status, S.CANCELLED)
        self.assertEqual(self.rail.steps, 100)
        self.assertFalse(self.carriage.homed)

    def test_phase_timeout_fails(self):
        self.build(VirtualRail(2000), phase_timeout_s=5)
        result = self.run_homing()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('timeout', result.error)

    def test_motor_fault_fails_and_invalidates(self):
        self.build(VirtualRail(2000, step_error_at=10))
        result = self.run_homing()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('fault', result.error)
        self.assertFalse(self.carriage.homed)

    def test_restart_while_running_rejected(self):
        self.build(VirtualRail(2000))
        self.homing.start()
        with self.assertRaises(RuntimeError):
            self.homing.start()

    def test_position_counts_only_output_steps_after_homing(self):
        self.build(VirtualRail(2000, right_at=1200))
        self.assertEqual(self.run_homing().status, S.DONE)
        self.assertEqual(self.move(1, 800).reason, R.COMPLETED)
        self.assertEqual(self.carriage.position_steps, 800)
        self.assertEqual(self.carriage.position_mm, 10.0)
        # 오른쪽 리미트에서 중단되면 실제 출력분만 반영 (물리 1200 = 원점 기준 800)
        self.assertEqual(self.move(1, 1000).reason, R.RIGHT_LIMIT)
        self.assertEqual(self.carriage.position_steps, 800)
        self.assertEqual(self.move(-1, 300).reason, R.COMPLETED)
        self.assertEqual(self.carriage.position_steps, 500)
        self.assertEqual(self.rail.position - 400, 500)

    def test_uncertain_pulse_invalidates_origin(self):
        self.build(VirtualRail(2000))
        self.assertEqual(self.run_homing().status, S.DONE)
        self.rail.step_error_at = self.rail.steps + 5
        self.assertEqual(self.move(1, 50).reason, R.FAULT)
        self.assertFalse(self.carriage.homed)
        self.assertIsNone(self.carriage.position_mm)

    def test_rehoming_after_fault_reset(self):
        self.build(VirtualRail(2000))
        self.run_homing()
        self.rail.step_error_at = self.rail.steps + 5
        self.move(1, 50)
        self.rail.step_error_at = None
        self.carriage.reset_fault()
        self.assertFalse(self.carriage.homed)
        self.assertEqual(self.run_homing().status, S.DONE)
        self.assertEqual(self.rail.position, 400)

    def test_set_home_rejected_while_moving(self):
        self.build(VirtualRail(2000))
        self.carriage.start_move(1, 10, speed_steps_s=100)
        with self.assertRaises(RuntimeError):
            self.carriage.set_home()

    def test_invalid_config(self):
        for overrides in ({'fast_speed_steps_s': 0}, {'slow_speed_steps_s': 200},
                          {'backoff_steps': 1.5}, {'max_reseek_steps': 160},
                          {'home_offset_mm': -1}, {'settle_s': 1000}, {'phase_timeout_s': float('nan')}):
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    config(**overrides)
        self.build(VirtualRail(2000))
        with self.assertRaises(ValueError):
            HomingProcedure(self.carriage, config(home_offset_mm=0.001))


if __name__ == '__main__':
    unittest.main()
