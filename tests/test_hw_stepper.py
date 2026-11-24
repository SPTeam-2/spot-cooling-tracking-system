"""
[파일(내부 구현 코드) 목적]
실제 모터 없이 STEP·DIR·EN 출력과 초기화·종료의 오류 처리를 자동 검증
[코드 내용]
FakePi·HardwareTests: 출력 순서, 단일 펄스, 핀 검증, 연결 실패, 부분 초기화 실패 검사
출력 실패 후 안전 레벨 시도, 반복 정리, 공유 연결 보존 검사
[플로우 간단 정리]
가상 pigpio 준비 → 모터 HW 생성 → 정상·실패 조건 주입 → 출력 호출·정리 결과 확인

모의 호출 검사이며 실제 펄스 폭·모터 회전·배선 검증을 대체하지 않음
"""
import unittest
from unittest.mock import patch
from src.carriage.hw_stepper import StepperHardware


class FakePi:
    connected = True
    def __init__(self):
        self.calls = []
    def write(self, pin, level):
        self.calls.append(('write', pin, level))
        return 0
    def set_mode(self, pin, mode):
        self.calls.append(('mode', pin, mode))
        return 0
    def stop(self):
        self.calls.append(('stop',))


class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.pi = FakePi()
        self.sleeps = []
        self.hw = StepperHardware(20,21,16,positive_dir_level=1,
                                  pi=self.pi, sleep=self.sleeps.append)

    def test_pulse_and_shared_cleanup(self):
        self.assertEqual(self.pi.calls[:2], [('write',16,1), ('mode',16,1)])
        self.hw.set_direction(-1)
        self.hw.enable()
        self.pi.calls.clear()
        self.hw.step_once()
        self.assertEqual(self.pi.calls, [('write',20,1), ('write',20,0)])
        self.hw.cleanup()
        calls = list(self.pi.calls)
        self.hw.cleanup()
        self.assertEqual(calls, self.pi.calls)
        self.assertNotIn(('stop',), calls)
        with self.assertRaises(ConnectionError):
            self.hw.enable()

    def test_high_failure_attempts_low(self):
        self.hw.set_direction(1)
        self.hw.enable()
        with patch.object(self.pi,'write', side_effect=[-1, 0]) as write:
            with self.assertRaises(OSError):
                self.hw.step_once()
            self.assertEqual(write.call_args_list[-1].args, (20,0))

    def test_disable_attempts_enable_even_when_step_fails(self):
        with patch.object(self.pi,'write', side_effect=[-1, 0]) as write:
            with self.assertRaises(OSError):
                self.hw.disable()
            self.assertEqual(write.call_args_list[-1].args, (16,1))

    def test_invalid_pins_before_output(self):
        for pins in ((5,21,16), (20,20,16), (20,21,True), (-1,21,16)):
            pi = FakePi()
            with self.assertRaises(ValueError):
                StepperHardware(*pins, positive_dir_level=1, pi=pi)
            self.assertEqual(pi.calls, [])

    def test_disconnected(self):
        self.pi.connected = False
        with self.assertRaises(ConnectionError):
            self.hw.set_direction(1)

    def test_partial_initialization_preserves_shared_connection(self):
        pi = FakePi()
        with patch.object(pi, 'set_mode', side_effect=[0, -1]):
            with self.assertRaises(OSError):
                StepperHardware(20,21,16,positive_dir_level=1,pi=pi)
        self.assertIn(('write',16,1), pi.calls)
        self.assertNotIn(('stop',), pi.calls)

    def test_failed_initial_write_still_attempts_safe_level(self):
        for failure_index in (0, 1):  # EN 초기 쓰기 실패와 STEP 초기 쓰기 실패를 각각 검사
            pi = FakePi()
            responses = [0] * failure_index + [-1] + [0] * 4
            with patch.object(pi, 'write', side_effect=responses) as write:
                with self.assertRaises(OSError):
                    StepperHardware(20, 21, 16, positive_dir_level=1, pi=pi)
                calls = [call.args for call in write.call_args_list]
            if failure_index == 0:
                self.assertEqual(calls, [(16, 1), (16, 1)])
            else:
                self.assertEqual(calls, [(16, 1), (20, 0), (20, 0), (16, 1)])
            self.assertNotIn(('stop',), pi.calls)
