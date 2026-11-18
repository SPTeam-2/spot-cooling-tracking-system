"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 Pi용 Homing 실행부의 설정 검사·사전 확인·Homing 진행·종료 정리를 자동 검증
[코드 내용]
RailDevice: 가상 레일 위치를 모터 출력과 가상 GPIO 리미트 입력이 함께 사용
HomingConfigTests: 테스트용 YAML의 빈 값·잘못된 키·속도 상한·Homing 조건 오류 거절
HomingRunnerTests: --home 없는 무이동, 정상 원점 확정, 양쪽 눌림, 스위치 미검출, Ctrl+C 취소, EN 미배선 거절
[플로우 간단 정리]
임시 YAML·가상 레일 준비 → parse_args() → run() → 종료 코드·가상 위치·정리 호출 확인

현장 실행부의 모의 검사이며 실제 Pi 라이브러리 호환·스위치 위치·기구 동작 검증은 별도 필요
"""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

path = Path(__file__).resolve().parents[1] / 'scripts' / 'carriage' / 'pi' / 'run_homing.py'
spec = importlib.util.spec_from_file_location('run_homing', path)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

STEPPER = 'direction: null\nsteps: null\nspeed: null\nmax_speed: 200\npositive_dir_level: 1\nio_timeout: 0.25\n'
HOMING = ('fast_speed_steps_s: 100\nslow_speed_steps_s: 50\nbackoff_steps: 160\nmax_seek_steps: 6000\n'
          'max_reseek_steps: 400\nphase_timeout_s: 1000\nsettle_s: 0.02\n')
COORDINATE = ('rail:\n  home_offset_mm: 5\nstepper:\n  microstep: 16\n  steps_per_rev: 200\n'
              '  belt_pitch_mm: 2\n  pulley_teeth: 20\n')


class RailDevice:
    """가상 레일. 위치 0 이하 왼쪽 눌림, right_at 이상 오른쪽 눌림 (스텝 단위)"""

    def __init__(self, position=2000, right_at=5000, left_works=True):
        self.position, self.right_at, self.left_works = position, right_at, left_works
        self.direction = 1
        self.steps = 0
        self.cleaned = False

    def level(self, pin):
        if pin == 5:
            return int(not (self.left_works and self.position <= 0))
        return int(not self.position >= self.right_at)

    # 모터 HW 인터페이스
    def check_connection(self): pass
    def set_direction(self, direction): self.direction = direction
    def enable(self): pass
    def disable(self): pass
    def cleanup(self): self.cleaned = True

    def step_once(self):
        self.position += self.direction
        self.steps += 1


class ConfigFiles:
    def __init__(self, testcase):
        directory = tempfile.TemporaryDirectory()
        testcase.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.write(STEPPER, HOMING, COORDINATE)

    def write(self, stepper=None, homing=None, coordinate=None):
        for name, text in (('stepper.yaml', stepper), ('homing.yaml', homing), ('coordinate.yaml', coordinate)):
            if text is not None:
                (self.root / name).write_text(text, encoding='utf-8')

    def argv(self, *extra):
        return ['--stepper-config', str(self.root / 'stepper.yaml'),
                '--homing-config', str(self.root / 'homing.yaml'),
                '--coordinate', str(self.root / 'coordinate.yaml'), *extra]


class HomingConfigTests(unittest.TestCase):
    def setUp(self):
        self.files = ConfigFiles(self)

    def parse(self, *extra):
        with contextlib.redirect_stderr(io.StringIO()):
            return runner.parse_args(self.files.argv(*extra))

    def test_valid_home_config(self):
        args = self.parse('--home')
        self.assertTrue(args.home)
        self.assertEqual(args.homing.home_offset_mm, 5)
        self.assertEqual((args.positive_dir_level, args.max_speed), (1, 200))

    def test_null_values_allow_preflight_only(self):
        self.files.write(homing=HOMING.replace('backoff_steps: 160', 'backoff_steps: null'))
        self.assertIsNone(self.parse().homing)
        with self.assertRaises(SystemExit):
            self.parse('--home')
        self.files.write(stepper=STEPPER.replace('positive_dir_level: 1', 'positive_dir_level: null'),
                         homing=HOMING)
        with self.assertRaises(SystemExit):
            self.parse('--home')

    def test_shipped_test_yaml_is_all_null(self):
        values = runner.load_homing_values(runner.ROOT / 'config' / 'homing_test.yaml')
        self.assertEqual(set(values), set(runner.HOMING_KEYS))
        self.assertTrue(all(value is None for value in values.values()))

    def test_invalid_files_and_values(self):
        cases = [
            {'homing': HOMING + 'unknown: 1\n'},
            {'homing': '[]'},
            {'homing': 'backoff_steps: ['},
            {'homing': HOMING.replace('fast_speed_steps_s: 100', 'fast_speed_steps_s: 300')},  # max_speed 초과
            {'homing': HOMING.replace('slow_speed_steps_s: 50', 'slow_speed_steps_s: 150')},  # slow > fast
            {'homing': HOMING.replace('backoff_steps: 160', 'backoff_steps: 1.5')},
            {'coordinate': 'stepper:\n  microstep: 16\n  steps_per_rev: 200\n  belt_pitch_mm: 2\n  pulley_teeth: 20\n'},
            {'coordinate': 'rail:\n  home_offset_mm: 5\n'},
        ]
        for case in cases:
            with self.subTest(case=case):
                self.files.write(STEPPER, HOMING, COORDINATE)
                self.files.write(**case)
                with self.assertRaises(SystemExit):
                    self.parse('--home')


class HomingRunnerTests(unittest.TestCase):
    def setUp(self):
        self.files = ConfigFiles(self)
        self.pins = {'stepper': {'step': 20, 'dir': 21, 'enable': 16},
                     'limit_switch': {'left': 5, 'right': 6}, 'fan': {'pwm': 18}}
        self.rail = RailDevice()
        self.gpio = Mock()
        self.gpio.input.side_effect = lambda pin: self.rail.level(pin)
        self.factory = Mock(side_effect=lambda *a, **k: self.rail)
        self.now = 0.0
        self.output = io.StringIO()
        contexts = contextlib.ExitStack()
        self.addCleanup(contexts.close)
        contexts.enter_context(contextlib.redirect_stdout(self.output))
        contexts.enter_context(contextlib.redirect_stderr(self.output))

    def tick(self):
        self.now += 0.011
        return self.now

    def run_args(self, *extra, sleep=None):
        args = runner.parse_args(self.files.argv(*extra))
        return runner.run(args, self.pins, gpio=self.gpio, hardware_factory=self.factory,
                          sleep=sleep or Mock(), clock=self.tick)

    def test_preflight_never_steps(self):
        self.assertEqual(self.run_args(), 0)
        self.assertEqual(self.rail.steps, 0)
        self.assertTrue(self.rail.cleaned)
        self.gpio.cleanup.assert_called_once_with((5, 6))
        self.assertIn('사전 확인 완료', self.output.getvalue())

    def test_home_sets_origin_and_cleans_up(self):
        self.assertEqual(self.run_args('--home'), 0)
        self.assertEqual(self.rail.position, 400)   # 2차 접촉점 0 + 5mm(400스텝)
        self.assertEqual(self.factory.call_args.kwargs['positive_dir_level'], 1)
        text = self.output.getvalue()
        for phase in ('SEEK_FAST', 'BACKOFF', 'SEEK_SLOW', 'OFFSET'):
            self.assertIn(f'단계: {phase}', text)
        self.assertIn('결과: DONE', text)
        self.assertIn('원점 확정: 현재 위치 0.0mm', text)
        self.assertTrue(self.rail.cleaned)
        self.gpio.cleanup.assert_called_once_with((5, 6))

    def test_home_from_left_pressed_releases_first(self):
        self.rail.position = -5
        self.assertEqual(self.run_args('--home'), 0)
        self.assertIn('단계: RELEASE', self.output.getvalue())
        self.assertEqual(self.rail.position, 400)

    def test_both_pressed_rejects_before_motor_output(self):
        self.rail.right_at = 0
        self.rail.position = 0
        self.assertEqual(self.run_args('--home'), 1)
        self.factory.assert_not_called()
        self.assertEqual(self.rail.steps, 0)
        self.gpio.cleanup.assert_called_once()

    def test_switch_not_found_fails(self):
        self.rail.left_works = False
        self.assertEqual(self.run_args('--home'), 1)
        self.assertIn('결과: FAILED', self.output.getvalue())
        self.assertEqual(self.rail.steps, 6000)
        self.assertTrue(self.rail.cleaned)

    def test_ctrl_c_cancels_without_origin(self):
        calls = {'n': 0}

        def sleep(_):
            calls['n'] += 1
            if calls['n'] == 50:
                raise KeyboardInterrupt

        self.assertEqual(self.run_args('--home', sleep=sleep), 130)
        steps = self.rail.steps
        self.assertLess(steps, 60)
        self.assertNotIn('원점 확정', self.output.getvalue())
        self.assertIn('STOP: 사용자 중단', self.output.getvalue())
        self.assertTrue(self.rail.cleaned)
        self.gpio.cleanup.assert_called_once()

    def test_cleanup_failure_sets_exit_code(self):
        self.rail.cleanup = Mock(side_effect=OSError('cleanup failed'))
        self.assertEqual(self.run_args('--home'), 1)
        self.gpio.cleanup.assert_called_once()

    def test_missing_enable_rejected_for_home(self):
        self.pins['stepper']['enable'] = None
        with self.assertRaises(ValueError):
            self.run_args('--home')
        self.gpio.setup.assert_not_called()


if __name__ == '__main__':
    unittest.main()
