"""pi_server가 리미트 스위치 차단·읽기 오류를 지키는지 검사 (GPIO 없이).

실행: python3 -m unittest discover -s tests -p test_pi_server_limits.py -v
"""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "comms"))
import pi_server
from carriage.function_limit_switch import LimitSwitchFunctions


class FakeReader:
    def __init__(self):
        self.levels = (1, 1)   # HIGH=안 눌림
        self.error = None

    def read_levels(self):
        if self.error:
            raise self.error
        return self.levels


class PiServerLimitTests(unittest.TestCase):
    def setUp(self):
        pi_server.ON_PI = False
        self.reader = FakeReader()
        self.act = pi_server.Actuators(limits=LimitSwitchFunctions(self.reader))
        self.act.current_mm = 300.0

    def move(self, target_mm):
        before = self.act.current_mm
        self.act.step_toward(target_mm, 0.01)
        return self.act.current_mm - before

    def test_moves_when_not_pressed(self):
        self.assertLess(self.move(0), 0)
        self.assertGreater(self.move(530), 0)

    def test_left_pressed_blocks_left_only(self):
        self.reader.levels = (0, 1)
        self.assertEqual(self.move(0), 0)
        self.assertGreater(self.move(530), 0)

    def test_right_pressed_blocks_right_only(self):
        self.reader.levels = (1, 0)
        self.assertEqual(self.move(530), 0)
        self.assertLess(self.move(0), 0)

    def test_both_pressed_blocks_both(self):
        self.reader.levels = (0, 0)
        self.assertEqual(self.move(0), 0)
        self.assertEqual(self.move(530), 0)

    def test_read_error_latches_fault(self):
        self.reader.error = OSError("read failed")
        self.assertEqual(self.move(0), 0)
        self.assertIn("read failed", self.act.fault)
        self.reader.error = None
        self.assertEqual(self.move(0), 0)   # 재시작 전까지 계속 정지


if __name__ == "__main__":
    unittest.main()
