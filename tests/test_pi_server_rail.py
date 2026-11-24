"""pi_server 레일 제어 검사: 구동부 실제 코드(Stepper·Homing·TargetMover) + 가상 레일 하드웨어.

실행: python3 -m unittest tests.test_pi_server_rail -v
"""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "comms"))
sys.path.insert(0, str(ROOT / "src"))
import pi_server
from protocol import Command
from comms.carriage_rail import CarriageRail, SimRail
from carriage.drive_geometry import DriveGeometry
from carriage.function_carriage import CarriageFunctions
from carriage.function_limit_switch import LimitSwitchFunctions
from carriage.function_stepper import StepperFunctions
from carriage.homing import HomingConfig, HomingProcedure
from carriage.target_move import TargetMoveConfig, TargetMover, TargetStatus

STEPS_PER_MM = 5       # 테스트용 풀스텝 기하 (2mm × 20T / 200)
OFFSET_STEPS = 25      # home_offset 5mm


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class SimRailHW:
    """스텝마다 위치가 바뀌는 가상 캐리지. 왼쪽 스위치는 0, 오른쪽은 right_at."""

    def __init__(self, start=2000, right_at=10_000):
        self.pos, self.right_at, self.dir = start, right_at, 1
        self.left_broken = False
        self.step_pin = 20

    def check_connection(self):
        pass

    def set_direction(self, d):
        self.dir = d

    def enable(self):
        pass

    def disable(self):
        pass

    def step_once(self):
        self.pos += self.dir

    def read_levels(self):
        left = self.pos <= 0 and not self.left_broken
        return (0 if left else 1, 0 if self.pos >= self.right_at else 1)


class FakeFan:
    def __init__(self):
        self.duty = 0

    def set(self, d):
        self.duty = d


class RailTests(unittest.TestCase):
    def make(self, travel_mm=200, **hw_kw):
        self.clock = Clock()
        self.hw = SimRailHW(**hw_kw)
        limits = LimitSwitchFunctions(self.hw)
        motor = StepperFunctions(self.hw, limits, max_speed_steps_s=1000, clock=self.clock)
        carriage = CarriageFunctions(motor, limits, DriveGeometry(2, 20, 200, 1))
        homing = HomingProcedure(carriage, HomingConfig(
            fast_speed_steps_s=800, slow_speed_steps_s=200, backoff_steps=20, max_seek_steps=5000,
            max_reseek_steps=40, home_offset_mm=5, phase_timeout_s=30, settle_s=0), clock=self.clock)
        mover = TargetMover(carriage, TargetMoveConfig(speed_steps_s=800, min_mm=0, max_mm=travel_mm))
        self.rail = CarriageRail(carriage, homing, mover, travel_mm=travel_mm, deadzone_mm=20)
        return self.rail

    def drive(self, target, seconds):
        for _ in range(int(seconds / 0.001)):
            self.clock.t += 0.001
            self.rail.tick(target)

    def home(self):
        self.drive(None, 10)
        self.assertEqual(self.rail.state, "준비", self.rail.fault)

    def test_homes_then_follows_target(self):
        self.make()
        self.home()
        self.assertEqual(self.rail.position_mm, 0)
        self.drive(100, 3)
        self.assertEqual(self.rail.position_mm, 100)
        self.assertEqual(self.hw.pos, OFFSET_STEPS + 100 * STEPS_PER_MM)

    def test_small_change_inside_deadzone_ignored(self):
        self.make()
        self.home()
        self.drive(100, 3)
        self.drive(110, 1)
        self.assertEqual(self.rail.position_mm, 100)
        self.drive(130, 1)
        self.assertEqual(self.rail.position_mm, 130)

    def test_retarget_while_moving(self):
        self.make()
        self.home()
        self.drive(180, 0.3)
        self.assertEqual(self.rail.mover.result.status, TargetStatus.MOVING)
        self.drive(40, 3)
        self.assertEqual(self.rail.position_mm, 40)

    def test_target_moving_away_does_not_restart_each_frame(self):
        self.make()
        self.home()
        calls = []
        orig = self.rail.mover.set_target
        self.rail.mover.set_target = lambda t: (calls.append(t), orig(t))[1]
        for i in range(160):                 # 사람이 걸어가듯 목표가 50ms마다 1mm씩 멀어짐 (20mm/s)
            self.drive(20 + i, 0.05)
        self.drive(179, 3)
        self.assertLessEqual(abs(self.rail.position_mm - 179), 20)   # 데드존 안까지 따라감
        self.assertLess(len(calls), 15)      # 매 프레임(160번) 재시작하지 않음

    def test_nearer_target_same_way_retargets_at_once(self):
        self.make()
        self.home()
        self.drive(180, 0.2)
        self.drive(60, 0.01)
        self.assertEqual(self.rail.mover.result.target_mm, 60)

    def test_none_stops_and_holds(self):
        self.make()
        self.home()
        self.drive(180, 0.5)
        self.drive(None, 0.1)
        stopped = self.rail.position_mm
        self.assertLess(stopped, 180)
        self.drive(None, 2)
        self.assertEqual(self.rail.position_mm, stopped)
        self.assertIsNone(self.rail.fault)

    def test_target_clamped_to_travel(self):
        self.make()
        self.home()
        self.drive(9999, 4)
        self.assertEqual(self.rail.position_mm, 200)
        self.drive(-50, 4)
        self.assertEqual(self.rail.position_mm, 0)

    def test_limit_hit_during_move_faults(self):
        self.make(right_at=OFFSET_STEPS + 150 * STEPS_PER_MM)   # travel_mm(200)보다 스위치가 앞에 있음
        self.home()
        self.drive(200, 4)
        self.assertIn("이동 실패", self.rail.fault)
        pos = self.hw.pos
        self.drive(0, 1)
        self.assertEqual(self.hw.pos, pos)                      # 재시작 전까지 안 움직임

    def test_homing_failure_faults(self):
        self.make()
        self.hw.left_broken = True
        self.drive(100, 10)
        self.assertIn("Homing", self.rail.fault)
        self.assertIsNone(self.rail.position_mm)


class ControlStepTests(unittest.TestCase):
    def setUp(self):
        self.t = [0.0]
        self.rail = SimRail(travel_mm=530, deadzone_mm=20, speed_mm_s=300, clock=lambda: self.t[0])
        self.fan = FakeFan()

    def step(self, cmd, linked, dt=0.01):
        self.t[0] += dt
        pi_server.control_step(self.rail, self.fan, cmd, linked)

    def test_follows_and_fans_when_active(self):
        cmd = Command(target_mm=300, fan_duty=70, active=True)
        for _ in range(200):
            self.step(cmd, True)
        self.assertEqual((self.rail.position_mm, self.fan.duty), (300, 70))

    def test_link_lost_stops_fan_and_motion(self):
        cmd = Command(target_mm=300, fan_duty=70, active=True)
        self.step(cmd, True)
        self.step(cmd, True)
        moved = self.rail.position_mm
        for _ in range(50):
            self.step(cmd, False)
        self.assertEqual((self.rail.position_mm, self.fan.duty), (moved, 0))

    def test_inactive_stops_fan(self):
        self.step(Command(target_mm=300, fan_duty=70, active=False), True)
        self.assertEqual(self.fan.duty, 0)

    def test_fault_stops_fan(self):
        self.rail.fault = "test"
        self.step(Command(target_mm=300, fan_duty=70, active=True), True)
        self.assertEqual(self.fan.duty, 0)


if __name__ == "__main__":
    unittest.main()
