"""Integrator 판단 로직 검사 (카메라·파이 없이).

실행: python3 -m unittest discover -s tests -p test_integrator.py -v
"""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.control.integrator import Integrator


def person(tid, rail_mm, shoulder_px=250.0, width_px=400.0):
    return SimpleNamespace(track_id=tid, rail_mm=rail_mm, shoulder_px=shoulder_px, width_px=width_px)


class IntegratorTests(unittest.TestCase):
    def setUp(self):
        self.integ = Integrator({})
        self.t = 0.0

    def run_for(self, seconds, obs):
        out = []
        for _ in range(int(seconds * 10)):
            self.t += 0.1
            out.append(self.integ.step(obs, self.t))
        return out

    def test_nobody_means_inactive(self):
        _, duty, active = self.integ.step([], 0.0)
        self.assertEqual((duty, active), (0, False))

    def test_target_position_and_duty_by_depth(self):
        for shoulder, duty in ((600.0, 40), (250.0, 70), (100.0, 100)):   # 417 / 1000 / 2500mm
            with self.subTest(shoulder=shoulder):
                integ = Integrator({})
                self.assertEqual(integ.step([person(1, 123.0, shoulder)], 0.0), (123.0, duty, True))

    def test_width_used_when_shoulder_missing(self):
        self.integ.step([person(1, 300.0, shoulder_px=None, width_px=250.0)], 0.0)
        self.assertEqual(self.integ.depth_mm, 1000.0)

    def test_single_person_never_dropped_by_cooldown(self):
        out = self.run_for(30, [person(1, 300.0)])
        self.assertTrue(all(active for _, _, active in out))

    def test_two_people_take_turns(self):
        a, b = person(1, 250.0), person(2, 350.0)
        self.run_for(1, [a, b])
        first = self.integ.target.wid
        self.run_for(10, [a, b])
        self.assertNotEqual(self.integ.target.wid, first)

    def test_lost_person_stops_fan(self):
        self.run_for(1, [person(1, 300.0)])
        out = self.run_for(3, [])
        self.assertEqual(out[-1][1:], (0, False))


if __name__ == "__main__":
    unittest.main()
