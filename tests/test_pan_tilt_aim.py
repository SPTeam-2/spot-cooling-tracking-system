"""팬틸트 연결 검사: 구동부 실제 코드(PanTiltFunctions·MoveToAngle) + 가짜 서보, 노트북 각도 계산, 프로토콜.

실행: python3 -m unittest tests.test_pan_tilt_aim -v
"""
from pathlib import Path
import json
import math
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "comms"))
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))
import pi_server
from protocol import Command
from comms.carriage_rail import SimRail
from comms.pan_tilt_aim import PanTiltAim, SimPanTilt, load_pan_tilt_settings
from pan_tilt.function_pan_tilt import AxisConfig, PanTiltFunctions
from pan_tilt.move_to_angle import MoveToAngle, MoveToAngleConfig
from src.control.integrator import Integrator


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeServo:
    def __init__(self):
        self.pulse_us, self.cleaned = None, False

    def set_pulse(self, us):
        self.pulse_us = us

    def off(self):
        self.pulse_us = 0

    def cleanup(self):
        self.cleaned = True


class FakeFan:
    def __init__(self):
        self.duty = 0

    def set(self, d):
        self.duty = d


class AimTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.pan_hw, self.tilt_hw = FakeServo(), FakeServo()
        pt = PanTiltFunctions(self.pan_hw, self.tilt_hw,
                              AxisConfig(1550, 10.0, -30, 30), AxisConfig(1200, -10.0, -20, 20))
        mover = MoveToAngle(pt, MoveToAngleConfig(max_speed_deg_s=60), clock=self.clock)
        self.aim = PanTiltAim(pt, mover, on_close=(self.pan_hw.cleanup, self.tilt_hw.cleanup))

    def drive(self, pan, tilt, seconds):
        for _ in range(int(seconds / 0.001)):
            self.clock.t += 0.001
            self.aim.tick(pan, tilt)

    def test_starts_centered(self):
        self.assertEqual(self.aim.angles, (0, 0))
        self.assertEqual((self.pan_hw.pulse_us, self.tilt_hw.pulse_us), (1550, 1200))

    def test_moves_to_commanded_angles(self):
        self.drive(20, 10, 1)
        self.assertEqual(self.aim.angles, (20, 10))
        self.assertEqual((self.pan_hw.pulse_us, self.tilt_hw.pulse_us), (1750, 1100))

    def test_out_of_range_is_clamped(self):
        self.drive(90, -90, 2)
        self.assertEqual(self.aim.angles, (30, -20))
        self.assertIsNone(self.aim.fault)

    def test_small_change_inside_deadband_ignored(self):
        self.drive(20, 10, 1)
        self.drive(20.4, 10.4, 0.5)
        self.assertEqual(self.aim.angles, (20, 10))

    def test_none_holds_then_resumes(self):
        self.drive(20, 0, 0.1)
        held = self.aim.angles
        self.drive(None, None, 0.5)
        self.assertEqual(self.aim.angles, held)
        self.drive(20, 0, 1)
        self.assertEqual(self.aim.angles, (20, 0))

    def test_close_turns_off_and_cleans(self):
        self.aim.close()
        self.assertEqual(self.pan_hw.pulse_us, 0)
        self.assertTrue(self.pan_hw.cleaned and self.tilt_hw.cleaned)


class ControlStepTests(unittest.TestCase):
    def setUp(self):
        self.t = [0.0]
        self.rail = SimRail(travel_mm=530, deadzone_mm=20, clock=lambda: self.t[0])
        self.fan, self.aim = FakeFan(), SimPanTilt()

    def test_angles_follow_when_running(self):
        pi_server.control_step(self.rail, self.fan, Command(300, 70, True, 0, 12, -5), True, self.aim)
        self.assertEqual(self.aim.angles, (12, -5))

    def test_link_lost_holds_angles(self):
        pi_server.control_step(self.rail, self.fan, Command(300, 70, True, 0, 12, -5), True, self.aim)
        pi_server.control_step(self.rail, self.fan, Command(300, 70, True, 0, -20, 9), False, self.aim)
        self.assertEqual(self.aim.angles, (12, -5))

    def test_without_pan_tilt_still_runs(self):
        pi_server.control_step(self.rail, self.fan, Command(300, 70, True), True)
        self.assertEqual(self.fan.duty, 70)


class SettingsTests(unittest.TestCase):
    def test_empty_config_disables_pan_tilt(self):
        settings, reason = load_pan_tilt_settings(ROOT / "config")
        if settings is None:
            self.assertIn("pan.center_pulse_us", reason)


class ProtocolTests(unittest.TestCase):
    def test_round_trip_with_angles(self):
        c = Command.from_bytes(Command(100, 40, True, 1.0, 12.5, -3).to_bytes())
        self.assertEqual((c.pan_deg, c.tilt_deg), (12.5, -3))

    def test_old_sender_without_angles(self):
        old = json.dumps({"target_mm": 100, "fan_duty": 40, "active": True, "ts": 1.0}).encode()
        c = Command.from_bytes(old)
        self.assertEqual((c.pan_deg, c.tilt_deg), (0, 0))


def obs(rail_mm, shoulder_px=343.0):
    return SimpleNamespace(track_id=1, rail_mm=rail_mm, width_px=600, shoulder_px=shoulder_px)


class IntegratorAimTests(unittest.TestCase):
    def cfg(self, **pt):
        return {"rail_range": [0, 600], "control": {"carriage_speed_mm_s": 10},
                "pan_tilt": {"mode": "auto", "fixed_pan_deg": 0, "fixed_tilt_deg": 5,
                             "fan_height_mm": 900, "aim_height_mm": 1200,
                             "max_pan_deg": 30, "max_tilt_deg": 30, **pt}}

    def test_fixed_mode_sends_fixed_angles(self):
        integ = Integrator(self.cfg(mode="fixed"))
        integ.step([obs(500)], 0.0)
        self.assertEqual((integ.pan_deg, integ.tilt_deg), (0, 5))

    def test_auto_tilt_up_to_chest(self):
        integ = Integrator(self.cfg())
        integ.step([obs(300)], 0.0)        # 어깨 343px → 1000mm, 캐리지 300mm (레일 가운데)
        self.assertAlmostEqual(integ.tilt_deg, math.degrees(math.atan2(300, 1000)), places=3)
        self.assertAlmostEqual(integ.pan_deg, 0, places=3)

    def test_auto_pan_toward_person_while_carriage_lags(self):
        integ = Integrator(self.cfg())
        integ.step([obs(500)], 0.0)        # 사람이 +쪽으로 200mm 앞서 있음
        self.assertAlmostEqual(integ.pan_deg, math.degrees(math.atan2(200, 1000)), places=3)
        integ.step([obs(100)], 0.1)
        self.assertLess(integ.pan_deg, 0)

    def test_auto_pan_clamped(self):
        integ = Integrator(self.cfg(max_pan_deg=5))
        integ.step([obs(600, shoulder_px=1000)], 0.0)
        self.assertEqual(integ.pan_deg, 5)

    def test_speed_from_config(self):
        integ = Integrator(self.cfg())
        integ.step([obs(500)], 0.0)
        integ.step([obs(500)], 1.0)
        self.assertAlmostEqual(integ.carriage_mm, 310)


if __name__ == "__main__":
    unittest.main()
