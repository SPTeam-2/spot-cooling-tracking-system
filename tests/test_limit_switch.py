"""
[파일(내부 구현 코드) 목적]
실제 Pi 없이 리미트 HW·function의 입력 해석·차단·오류 처리 자동 검증
[코드 내용]
FakeGPIO와 unittest: 입력 조합·방향·잘못된 값·GPIO 오류·정리 시험
[플로우 간단 정리]
테스트용 가상 GPIO 준비 → 각 조건 입력 → 반환값·오류·정리 확인 → unittest 결과 출력
실제 GPIO 없이 리미트 입력·방향 차단·오류 전달 검증

실행: python -m unittest discover -s tests -p test_limit_switch.py -v
"""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.carriage.hw_limit_switch import LimitSwitchHardware
from src.carriage.function_limit_switch import LimitSwitchFunctions



class FakeGPIO:
    BCM, IN, PUD_UP = "BCM", "IN", "UP"

    def __init__(self):
        self.levels = {5: 1, 6: 1}
        self.setup_calls = []
        self.cleaned = []

    def setmode(self, mode):
        self.mode = mode

    def setup(self, pin, mode, **kwargs):
        self.setup_calls.append((pin, mode, kwargs))

    def input(self, pin):
        return self.levels[pin]

    def cleanup(self, pins):
        self.cleaned.append(pins)


class LimitSwitchTests(unittest.TestCase):
    def setUp(self):
        self.gpio = FakeGPIO()
        self.hw = LimitSwitchHardware(5, 6, gpio=self.gpio)
        self.functions = LimitSwitchFunctions(self.hw)
        self.addCleanup(self.hw.cleanup)

    def test_four_states_and_both_directions(self):
        for left, right in ((False, False), (True, False), (False, True), (True, True)):
            with self.subTest(left=left, right=right):
                self.gpio.levels = {5: int(not left), 6: int(not right)}
                state = self.functions.read_state()
                self.assertEqual((state.left_pressed, state.right_pressed), (left, right))
                self.assertEqual(state.both_pressed, left and right)
                self.assertEqual(self.functions.is_blocked(-1), left)
                self.assertEqual(self.functions.is_blocked(1), right)

    def test_reads_fresh_input(self):
        self.assertFalse(self.functions.is_blocked(-1))
        self.gpio.levels[5] = 0
        self.assertTrue(self.functions.is_blocked(-1))
        self.gpio.levels[5] = 1
        self.assertFalse(self.functions.is_blocked(-1))

    def test_invalid_directions(self):
        for direction in (0, 2, True, -1.0, "left", None):
            with self.subTest(direction=direction), self.assertRaises(ValueError):
                self.functions.is_blocked(direction)

    def test_invalid_levels_are_not_silently_allowed(self):
        for level in (None, "0", 2, -1, 0.0):
            self.gpio.levels[5] = level
            with self.subTest(level=level), self.assertRaises(ValueError):
                self.functions.is_blocked(-1)

    def test_gpio_read_error_propagates(self):
        with patch.object(self.gpio, "input", side_effect=OSError("read failed")):
            with self.assertRaisesRegex(OSError, "read failed"):
                self.functions.is_blocked(1)

    def test_pullup_setup_and_pin_specific_cleanup(self):
        self.assertEqual(self.gpio.mode, "BCM")
        self.assertEqual(self.gpio.setup_calls,
                         [(5, "IN", {"pull_up_down": "UP"}),
                          (6, "IN", {"pull_up_down": "UP"})])
        self.hw.cleanup()
        self.hw.cleanup()
        self.assertEqual(self.gpio.cleaned, [(5, 6)])
        with self.assertRaises(RuntimeError):
            self.hw.read_levels()

    def test_invalid_pins_do_not_touch_gpio(self):
        gpio = FakeGPIO()
        for pins in ((5, 5), (-1, 6), (5, 28), (True, 6), (5.0, 6)):
            with self.subTest(pins=pins), self.assertRaises(ValueError):
                LimitSwitchHardware(*pins, gpio=gpio)
        self.assertEqual(gpio.setup_calls, [])

    def test_partial_setup_failure_releases_only_successful_pin(self):
        gpio = FakeGPIO()
        with patch.object(gpio, "setup", side_effect=[None, OSError("setup failed")]):
            with self.assertRaisesRegex(OSError, "setup failed"):
                LimitSwitchHardware(5, 6, gpio=gpio)
        self.assertEqual(gpio.cleaned, [(5,)])

if __name__ == "__main__":
    unittest.main()
