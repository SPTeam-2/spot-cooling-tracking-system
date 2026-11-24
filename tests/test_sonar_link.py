"""초음파 연결 검사: Sonar 중앙값·유효 범위, Status 왕복, 파이 → 노트북 UDP 회신, 깊이 융합.

실행: python3 -m unittest tests.test_sonar_link -v
"""
from pathlib import Path
import socket
import time
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "comms"))
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))
import pc_sender
import pi_server
from protocol import Status
from comms.sonar import Sonar, SimSonar
from src.control.integrator import Integrator


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class SonarTests(unittest.TestCase):
    def make(self, values):
        self.clock = Clock()
        it = iter(values)
        self.sonar = Sonar(lambda: next(it), clock=self.clock)
        for _ in values:
            self.sonar.sample()
            self.clock.t += 0.06

    def test_median_of_recent(self):
        self.make([1000, 1010, 3000, 990, 1005])
        self.assertEqual(self.sonar.read_mm(), 1005)

    def test_out_of_range_and_no_echo_ignored(self):
        self.make([None, 5, 9000, 800])
        self.assertEqual(self.sonar.read_mm(), 800)

    def test_stale_is_none(self):
        self.make([1000])
        self.clock.t += 1.0
        self.assertIsNone(self.sonar.read_mm())

    def test_nothing_yet_is_none(self):
        self.make([None, None])
        self.assertIsNone(self.sonar.read_mm())


class StatusTests(unittest.TestCase):
    def test_round_trip(self):
        self.assertEqual(Status.from_bytes(Status(1234.5, 1.0).to_bytes()).sonar_mm, 1234.5)

    def test_no_reading(self):
        self.assertIsNone(Status.from_bytes(Status(None, 1.0).to_bytes()).sonar_mm)


class UdpReturnTests(unittest.TestCase):
    def setUp(self):
        self.pi = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pi.bind(("127.0.0.1", 0))
        self.pi.settimeout(1.0)
        self.link = pc_sender.PiLink("127.0.0.1", self.pi.getsockname()[1])

    def tearDown(self):
        self.pi.close()
        self.link.sock.close()

    def reply(self, sonar):
        self.link.send(300, 70, True)
        _, addr = self.pi.recvfrom(1024)
        pi_server.send_status(self.pi, addr, sonar)
        for _ in range(100):
            if self.link.sonar_mm() is not None:
                break
            time.sleep(0.01)

    def test_pi_replies_to_sender(self):
        self.reply(SimSonar(950))
        self.assertEqual(self.link.sonar_mm(), 950)

    def test_old_status_ignored(self):
        self.reply(SimSonar(950))
        self.assertIsNone(self.link.sonar_mm(now=self.link._status_rx + 1.0))

    def test_no_status_is_none(self):
        self.assertIsNone(self.link.sonar_mm())


def obs(shoulder_px=343.0):
    return SimpleNamespace(track_id=1, rail_mm=300, width_px=600, shoulder_px=shoulder_px)


class FusionTests(unittest.TestCase):
    def test_weight_from_config(self):
        integ = Integrator({"rail_range": [0, 600], "depth": {"sonar_weight": 0.7}})
        integ.step([obs()], 0.0, sonar_mm=1100)     # 0.3*1000 + 0.7*1100
        self.assertAlmostEqual(integ.depth_mm, 1070)

    def test_gap_from_config(self):
        integ = Integrator({"rail_range": [0, 600], "depth": {"max_gap_mm": 50}})
        integ.step([obs()], 0.0, sonar_mm=1100)
        self.assertAlmostEqual(integ.depth_mm, 1000)

    def test_repo_config_is_half_half(self):
        import yaml
        with open(ROOT / "config" / "coordinate.yaml", encoding="utf-8") as f:
            integ = Integrator(yaml.safe_load(f))
        integ.step([obs()], 0.0, sonar_mm=1100)
        self.assertAlmostEqual(integ.depth_mm, 1050)

    def test_sonar_averaged_with_camera(self):
        integ = Integrator({"rail_range": [0, 600]})
        integ.step([obs()], 0.0, sonar_mm=1100)     # 카메라 1000mm
        self.assertAlmostEqual(integ.depth_mm, 1050)

    def test_far_off_sonar_dropped(self):
        integ = Integrator({"rail_range": [0, 600]})
        integ.step([obs()], 0.0, sonar_mm=300)      # 벽·사물로 보고 버림
        self.assertAlmostEqual(integ.depth_mm, 1000)

    def test_sonar_changes_fan_level(self):
        integ = Integrator({"rail_range": [0, 600]})
        integ.step([obs(shoulder_px=343 * 1000 / 1150)], 0.0)    # 카메라만 1150mm → 약풍
        self.assertEqual(integ.duty, 70)
        integ2 = Integrator({"rail_range": [0, 600]})
        integ2.step([obs(shoulder_px=343 * 1000 / 1150)], 0.0, sonar_mm=1450)   # 평균 1300 → 강풍
        self.assertEqual(integ2.duty, 100)


if __name__ == "__main__":
    unittest.main()
