"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 가상 레일로 목표 위치(mm) 이동 알고리즘을 자동 검증
[코드 내용]
VirtualRail: 모터 위치와 좌우 스위치 입력이 같은 위치값을 공유하는 가상 장치 (펄스 오류 재현)
TargetMoveTests: 도착·양방향 이동·거절·목표 변경·STOP·리미트 접촉·모터 오류·반복 이동 누적 오차 검사
[플로우 간단 정리]
가상 레일 400스텝 지점을 원점으로 확정 → set_target() → update() 반복 → 상태·가상 위치·추정 위치 확인

알고리즘 확인용 PC 검사이며 실제 이동 거리·탈조·벨트 미끄러짐 검증을 대체하지 않음
"""
import math
import unittest
from src.carriage.drive_geometry import DriveGeometry
from src.carriage.function_carriage import CarriageFunctions
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions, StepperReason as R
from src.carriage.homing import HomingConfig, HomingProcedure, HomingStatus
from src.carriage.target_move import TargetMoveConfig, TargetMover, TargetStatus as S

HOME = 400   # 가상 레일에서 원점(0mm)이 되는 물리 위치(스텝)


class VirtualRail:
    """위치 단위는 스텝. 0 이하 왼쪽 눌림, right_at 이상 오른쪽 눌림"""

    def __init__(self, position=HOME, *, right_at=50000, step_error_at=None):
        self.position = position
        self.direction = 1
        self.right_at = right_at
        self.step_error_at = step_error_at
        self.steps = 0

    def read_levels(self):
        return int(not self.position <= 0), int(not self.position >= self.right_at)   # 눌림=LOW(0)

    def check_connection(self): pass
    def set_direction(self, direction): self.direction = direction
    def enable(self): pass
    def disable(self): pass

    def step_once(self):
        if self.step_error_at is not None and self.steps == self.step_error_at:
            raise OSError('virtual pulse error')
        self.position += self.direction
        self.steps += 1


class TargetMoveTests(unittest.TestCase):
    def build(self, rail=None, *, home=True, **config):
        self.rail = rail or VirtualRail()
        self.now = 0.0
        clock = lambda: self.now
        limits = LimitSwitchFunctions(self.rail)
        self.motor = StepperFunctions(self.rail, limits, max_speed_steps_s=100, clock=clock)
        self.carriage = CarriageFunctions(self.motor, limits, DriveGeometry(2, 20, 200, 16))   # 80 steps/mm
        if home:
            self.carriage.set_home()
        values = dict(speed_steps_s=100, min_mm=0, max_mm=530)
        values.update(config)
        self.mover = TargetMover(self.carriage, TargetMoveConfig(**values))

    def run_until_done(self, limit=200000, stop_at_step=None):
        result = self.mover.result
        for _ in range(limit):
            if result.status != S.MOVING:
                return result
            self.now += 0.011
            result = self.mover.update(stop_requested=self.rail.steps == stop_at_step)
        self.fail('iteration bound exceeded')

    def advance(self, count):
        """이동 도중까지만 update()를 count번 진행"""
        for _ in range(count):
            self.now += 0.011
            self.mover.update()

    def physical_mm(self):
        return (self.rail.position - HOME) / 80

    def test_arrives_at_target(self):
        self.build()
        self.assertEqual(self.mover.set_target(100).status, S.MOVING)
        result = self.run_until_done()
        self.assertEqual(result.status, S.ARRIVED)
        self.assertEqual(result.target_mm, 100)
        self.assertEqual(result.position_mm, 100.0)
        self.assertEqual(self.rail.position, HOME + 8000)

    def test_moves_toward_motor_side(self):
        self.build()
        self.mover.set_target(100)
        self.run_until_done()
        self.mover.set_target(20.5)
        result = self.run_until_done()
        self.assertEqual(result.status, S.ARRIVED)
        self.assertEqual(self.rail.direction, -1)
        self.assertEqual(self.carriage.position_steps, 1640)
        self.assertEqual(self.physical_mm(), 20.5)

    def test_same_position_arrives_without_pulse(self):
        self.build()
        self.assertEqual(self.mover.set_target(0).status, S.ARRIVED)
        self.assertEqual(self.mover.set_target(0.004).status, S.ARRIVED)   # 반 스텝 미만은 0스텝
        self.assertEqual(self.rail.steps, 0)

    def test_rejects_before_home(self):
        self.build(home=False)
        with self.assertRaises(RuntimeError):
            self.mover.set_target(100)
        self.assertEqual(self.mover.result.status, S.IDLE)
        self.assertIsNone(self.mover.result.position_mm)
        self.assertEqual(self.rail.steps, 0)

    def test_rejects_invalid_and_out_of_range_targets(self):
        self.build()
        for bad in (math.nan, math.inf, -math.inf, '100', None, True):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.mover.set_target(bad)
        for bad in (-0.1, 530.01, 1000):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.mover.set_target(bad)
        self.assertEqual(self.mover.set_target(530).status, S.MOVING)   # 끝값은 허용

    def test_rejected_target_keeps_current_move(self):
        self.build()
        self.mover.set_target(50)
        self.advance(100)
        with self.assertRaises(ValueError):
            self.mover.set_target(999)
        self.assertEqual(self.mover.result.status, S.MOVING)
        self.assertEqual(self.mover.result.target_mm, 50)
        self.assertEqual(self.run_until_done().status, S.ARRIVED)
        self.assertEqual(self.physical_mm(), 50)

    def test_new_target_while_moving_goes_directly(self):
        self.build()
        self.mover.set_target(100)
        self.advance(3000)                                 # 도중에 새 목표
        self.assertEqual(self.mover.result.status, S.MOVING)
        midway = self.rail.position
        self.assertLess(midway, HOME + 8000)
        self.mover.set_target(10)
        self.assertEqual(self.rail.direction, -1)         # 원점까지 가지 않고 바로 방향 전환
        result = self.run_until_done()
        self.assertEqual(result.status, S.ARRIVED)
        self.assertEqual(self.physical_mm(), 10)
        self.assertGreater(min(self.rail.position, midway), HOME)

    def test_same_target_while_moving_does_not_restart(self):
        self.build()
        self.mover.set_target(100)
        self.advance(500)
        requested = self.motor.result.requested_steps
        self.mover.set_target(100.001)                     # 같은 스텝 목표
        self.assertEqual(self.motor.result.requested_steps, requested)
        self.assertEqual(self.run_until_done().status, S.ARRIVED)

    def test_stop_cancels_and_keeps_home(self):
        self.build()
        self.mover.set_target(100)
        result = self.run_until_done(stop_at_step=1234)
        self.assertEqual(result.status, S.CANCELLED)
        self.assertTrue(self.carriage.homed)
        self.assertEqual(self.carriage.position_steps, 1234)   # 요청이 아닌 실제 출력 스텝만 반영
        self.assertEqual(self.rail.steps, 1234)
        self.now += 10
        self.mover.update()
        self.assertEqual(self.rail.steps, 1234)                 # 자동 재개 없음
        self.mover.set_target(30)
        self.assertEqual(self.run_until_done().status, S.ARRIVED)
        self.assertEqual(self.physical_mm(), 30)

    def test_external_stop_is_cancelled(self):
        self.build()
        self.mover.set_target(100)
        self.advance(100)
        self.carriage.stop()
        self.assertEqual(self.mover.update().status, S.CANCELLED)
        self.assertTrue(self.carriage.homed)

    def test_limit_during_move_fails_and_invalidates_home(self):
        self.build(VirtualRail(right_at=HOME + 4000))           # 실제보다 레일이 짧은 상황
        self.mover.set_target(100)
        result = self.run_until_done()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('right_limit', result.error)
        self.assertFalse(self.carriage.homed)
        self.assertIsNone(result.position_mm)
        self.assertEqual(self.rail.position, HOME + 4000)       # 접촉 후 추가 펄스 없음
        with self.assertRaises(RuntimeError):
            self.mover.set_target(10)                            # 다시 Homing 전에는 거절

    def test_limit_pressed_at_start_fails(self):
        self.build()
        self.mover.set_target(1)
        self.run_until_done()
        self.rail.position = 0                                   # 탈조로 실제 캐리지가 왼쪽 스위치에 닿은 상황
        result = self.mover.set_target(0)                        # -방향 이동이 시작부터 차단됨
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('left_limit', result.error)
        self.assertFalse(self.carriage.homed)
        self.assertEqual(self.rail.position, 0)

    def test_motor_fault_fails_and_invalidates_home(self):
        self.build(VirtualRail(step_error_at=100))
        self.mover.set_target(100)
        result = self.run_until_done()
        self.assertEqual(result.status, S.FAILED)
        self.assertIn('motor fault', result.error)
        self.assertFalse(self.carriage.homed)
        self.assertEqual(self.motor.result.reason, R.FAULT)

    def test_repeated_targets_keep_zero_accumulated_error(self):
        self.build()
        for target in (100, 12.3, 530, 0, 250.5, 75, 499.99, 1, 300, 0.0125):
            with self.subTest(target=target):
                self.mover.set_target(target)
                self.assertEqual(self.run_until_done().status, S.ARRIVED)
                self.assertEqual(self.rail.position - HOME, self.carriage.position_steps)
                self.assertEqual(self.carriage.position_steps, self.carriage.geometry.mm_to_steps(target))

    def test_after_homing_procedure(self):
        self.build(VirtualRail(position=2000), home=False)
        homing = HomingProcedure(self.carriage, HomingConfig(
            fast_speed_steps_s=100, slow_speed_steps_s=50, backoff_steps=160, max_seek_steps=6000,
            max_reseek_steps=400, home_offset_mm=5, phase_timeout_s=1000, settle_s=0.02),
            clock=lambda: self.now)
        result = homing.start()
        while result.status == HomingStatus.RUNNING:
            self.now += 0.011
            result = homing.update()
        self.assertEqual(result.status, HomingStatus.DONE)
        self.assertEqual(self.rail.position, HOME)
        self.mover.set_target(42)
        self.assertEqual(self.run_until_done().status, S.ARRIVED)
        self.assertEqual(self.physical_mm(), 42)

    def test_invalid_config(self):
        for values in (dict(speed_steps_s=0, min_mm=0, max_mm=530),
                       dict(speed_steps_s=math.nan, min_mm=0, max_mm=530),
                       dict(speed_steps_s=100, min_mm=-1, max_mm=530),
                       dict(speed_steps_s=100, min_mm=10, max_mm=10),
                       dict(speed_steps_s=100, min_mm=0, max_mm=math.inf),
                       dict(speed_steps_s='100', min_mm=0, max_mm=530)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                TargetMoveConfig(**values)

    def test_speed_above_motor_limit_fails_without_pulse(self):
        self.build(speed_steps_s=1000)                           # 모터 상한 100 steps/s 초과
        with self.assertRaises(ValueError):
            self.mover.set_target(10)
        self.assertEqual(self.rail.steps, 0)
        self.assertEqual(self.mover.result.status, S.FAILED)
        self.assertTrue(self.carriage.homed)                     # 이동 전 거절이므로 원점 유지


if __name__ == '__main__':
    unittest.main()
