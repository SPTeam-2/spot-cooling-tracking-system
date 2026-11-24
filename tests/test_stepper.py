"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 모터 이동·리미트 차단·STOP·오류 잠금 동작을 자동 검증
[코드 내용]
StepperTests: 가상 모터·리미트·시계를 사용해 정확한 출력 수, 후퇴, 자동 재개 방지 검사
양쪽 눌림·입력 오류·출력 오류·잘못된 요청·역행 시계 및 위치 불확실 표시 검사
[플로우 간단 정리]
가상 장치·시계 준비 → 이동 요청 → 입력·시간·오류 변경 → update() → 상태·펄스 수 확인

기능 계층의 판단 검사이며 실제 정지거리·토크·탈조 검증을 대체하지 않음
"""
import unittest
from unittest.mock import Mock
from src.carriage.function_stepper import StepperFunctions, StepperReason as R
from src.carriage.function_limit_switch import LimitSwitchFunctions


class StepperTests(unittest.TestCase):
    def setUp(self):
        self.hw = Mock()
        self.reader = Mock()
        self.reader.read_levels.return_value = (1,1)
        self.now = 0.0
        self.motor = StepperFunctions(self.hw,LimitSwitchFunctions(self.reader),
                                     max_speed_steps_s=100, clock=lambda:self.now)

    def start(self, direction=1, steps=3):
        return self.motor.start_move(direction,steps,speed_steps_s=10)

    def test_exact_count_and_no_catchup(self):
        self.start()
        for t in (0,100,200):
            self.now=t
            self.motor.update()
        self.assertEqual(self.motor.result.reason, R.COMPLETED)
        self.motor.update()
        self.assertEqual(self.hw.step_once.call_count,3)

    def test_limit_before_start_and_retreat(self):
        self.reader.read_levels.return_value=(0,1)
        self.assertEqual(self.start(-1).reason,R.LEFT_LIMIT)
        self.hw.step_once.assert_not_called()
        self.start(1)
        self.motor.update()
        self.hw.step_once.assert_called_once()

    def test_limit_during_move_checks_even_before_due(self):
        self.start()
        self.motor.update()
        self.reader.read_levels.return_value=(1,0)
        self.assertEqual(self.motor.update().reason,R.RIGHT_LIMIT)
        self.reader.read_levels.return_value=(1,1)
        self.motor.update()
        self.hw.step_once.assert_called_once()

    def test_stop_and_no_auto_resume(self):
        self.start()
        self.assertEqual(self.motor.update(True).reason,R.STOPPED)
        self.now=100
        self.motor.update()
        self.hw.step_once.assert_not_called()

    def test_both_latches_fault(self):
        self.reader.read_levels.return_value=(0,0)
        self.assertEqual(self.start().reason,R.FAULT)
        self.assertEqual(self.motor.reset_fault().reason,R.FAULT)
        self.reader.read_levels.return_value=(1,1)
        with self.assertRaises(RuntimeError):
            self.start()
        self.assertEqual(self.motor.reset_fault().reason,R.IDLE)
        self.hw.step_once.assert_not_called()

    def test_read_error_and_bad_level(self):
        for value in ((-1,1),(2,1)):
            self.setUp()
            self.start()
            self.reader.read_levels.return_value=value
            self.assertEqual(self.motor.update().reason,R.FAULT)
            self.hw.step_once.assert_not_called()
        self.setUp()
        self.start()
        self.reader.read_levels.side_effect=OSError('read failed')
        self.assertEqual(self.motor.update().reason,R.FAULT)

    def test_uncertain_pulse_not_counted(self):
        self.start()
        self.hw.step_once.side_effect=OSError('LOW failed')
        result=self.motor.update()
        self.assertEqual(result.reason,R.FAULT)
        self.assertTrue(result.position_uncertain)
        self.assertEqual(result.completed_steps,0)
        self.motor.update()
        self.hw.step_once.assert_called_once()

    def test_reject_requests(self):
        for direction,steps,speed in ((True,1,10),(1,1.5,10),(1,0,10),
                                      (1,1,float('nan')),(1,1,101)):
            with self.assertRaises(ValueError):
                self.motor.start_move(direction,steps,speed_steps_s=speed)
        self.hw.enable.assert_not_called()
        self.start()
        with self.assertRaises(RuntimeError):
            self.start()

    def test_backwards_clock(self):
        self.start()
        self.now=-1
        self.assertEqual(self.motor.update().reason,R.FAULT)
        self.hw.step_once.assert_not_called()


class PulsePlanTests(unittest.TestCase):
    def test_ramp_counts_inside_total(self):
        from src.carriage.function_stepper import build_pulse_plan, plan_duration_s
        plan = build_pulse_plan(1600, 250)
        self.assertEqual(plan, [(0, 160), (40, 200), (80, 250), (1520, 200), (1560, 160)])
        self.assertAlmostEqual(plan_duration_s(plan, 1600), 80/160 + 80/200 + 1440/250)

    def test_short_moves_drop_highest_levels(self):
        from src.carriage.function_stepper import build_pulse_plan
        self.assertEqual(build_pulse_plan(150, 250), [(0, 160), (40, 200), (110, 160)])
        self.assertEqual(build_pulse_plan(80, 250), [(0, 160)])
        self.assertEqual(build_pulse_plan(1, 200), [(0, 160)])
        self.assertEqual(build_pulse_plan(500, 160), [(0, 160)])
        self.assertEqual(build_pulse_plan(500, 100), [(0, 100)])   # 시작 주파수보다 느리면 가감속 없음

    def test_mode_selection(self):
        from src.carriage.function_stepper import StepperFunctions
        limits = LimitSwitchFunctions(Mock())
        self.assertEqual(StepperFunctions(Mock(), limits, max_speed_steps_s=100).pulse_mode, 'step')
        with self.assertRaises(ValueError):
            StepperFunctions(Mock(), limits, max_speed_steps_s=100, pulse_mode='wave')
