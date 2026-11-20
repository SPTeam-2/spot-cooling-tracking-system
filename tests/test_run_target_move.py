"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 Pi용 목표 이동 실행부의 설정 검사·사전 확인·Homing 후 목표 이동·종료 정리를 자동 검증
[코드 내용]
RailDevice: 가상 레일 위치를 모터 출력과 가상 GPIO 리미트 입력이 함께 사용
TargetConfigTests: 테스트용 YAML의 빈 값·잘못된 키·목표 형식·범위 밖 목표·속도 상한·명령줄 우선 검사
TargetRunnerTests: --move 없는 무이동, Homing 후 목표 차례 이동, Enter 대기·종료, 리미트 접촉 실패, Homing 실패, Ctrl+C, 정리 실패
[플로우 간단 정리]
임시 YAML·가상 레일 준비 → parse_args() → run() → 종료 코드·가상 위치·출력·정리 호출 확인

현장 실행부의 모의 검사이며 실제 이동 거리·Pi 라이브러리 호환·기구 동작 검증은 별도 필요
"""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

path = Path(__file__).resolve().parents[1] / 'scripts' / 'carriage' / 'pi' / 'run_target_move.py'
sys.path.insert(0, str(path.parent))
spec = importlib.util.spec_from_file_location('run_target_move', path)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

STEPPER = 'direction: null\nsteps: null\nspeed: null\nmax_speed: 200\npositive_dir_level: 1\nio_timeout: 0.25\n'
HOMING = ('fast_speed_steps_s: 100\nslow_speed_steps_s: 50\nbackoff_steps: 160\nmax_seek_steps: 60000\n'
          'max_reseek_steps: 400\nphase_timeout_s: 100000\nsettle_s: 0.02\n')
TARGET = 'speed_steps_s: 200\ntargets_mm: [50, 10.5, 0]\n'
COORDINATE = ('rail:\n  home_offset_mm: 5\n  travel_mm: 530\nstepper:\n  microstep: 16\n  steps_per_rev: 200\n'
              '  belt_pitch_mm: 2\n  pulley_teeth: 20\n')
HOME = 400


class RailDevice:
    """가상 레일. 위치 0 이하 왼쪽 눌림, right_at 이상 오른쪽 눌림 (스텝 단위)"""

    def __init__(self, position=2000, right_at=50000, left_works=True):
        self.position, self.right_at, self.left_works = position, right_at, left_works
        self.direction = 1
        self.steps = 0
        self.cleaned = False

    def level(self, pin):
        if pin == 5:
            return int(not (self.left_works and self.position <= 0))
        return int(not self.position >= self.right_at)

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
        self.write(STEPPER, HOMING, TARGET, COORDINATE)

    def write(self, stepper=None, homing=None, target=None, coordinate=None):
        for name, text in (('stepper.yaml', stepper), ('homing.yaml', homing),
                           ('target.yaml', target), ('coordinate.yaml', coordinate)):
            if text is not None:
                (self.root / name).write_text(text, encoding='utf-8')

    def argv(self, *extra):
        return ['--stepper-config', str(self.root / 'stepper.yaml'),
                '--homing-config', str(self.root / 'homing.yaml'),
                '--target-config', str(self.root / 'target.yaml'),
                '--coordinate', str(self.root / 'coordinate.yaml'), *extra]


class TargetConfigTests(unittest.TestCase):
    def setUp(self):
        self.files = ConfigFiles(self)

    def parse(self, *extra):
        with contextlib.redirect_stderr(io.StringIO()):
            return runner.parse_args(self.files.argv(*extra))

    def test_valid_move_config_and_cli_override(self):
        args = self.parse('--move')
        self.assertEqual(args.targets, [50.0, 10.5, 0.0])
        self.assertEqual((args.target.speed_steps_s, args.target.max_mm), (200, 530))
        self.assertEqual(args.homing.home_offset_mm, 5)
        args = self.parse('--move', '--speed', '150', '--targets', '100', '530')
        self.assertEqual(args.targets, [100.0, 530.0])
        self.assertEqual(args.target.speed_steps_s, 150)

    def test_null_values_allow_preflight_only(self):
        self.files.write(target='speed_steps_s: null\ntargets_mm: null\n')
        self.assertIsNone(self.parse().target)
        with self.assertRaises(SystemExit):
            self.parse('--move')
        self.assertEqual(self.parse('--move', '--speed', '100', '--targets', '20').targets, [20.0])

    def test_shipped_test_yaml_is_all_null(self):
        values = runner.load_target_values(runner.ROOT / 'config' / 'target_move_test.yaml')
        self.assertEqual(set(values), set(runner.TARGET_KEYS))
        self.assertTrue(all(value is None for value in values.values()))

    def test_invalid_files_and_values(self):
        cases = [
            ({'target': TARGET + 'unknown: 1\n'}, ()),
            ({'target': 'speed_steps_s: 200\ntargets_mm: 50\n'}, ()),            # 목록 아님
            ({'target': 'speed_steps_s: 200\ntargets_mm: [50, "a"]\n'}, ()),
            ({'target': 'speed_steps_s: 200\ntargets_mm: []\n'}, ()),
            ({'target': 'speed_steps_s: 200\ntargets_mm: [50, 531]\n'}, ()),     # 범위 밖
            ({'target': 'speed_steps_s: 200\ntargets_mm: [-1]\n'}, ()),
            ({'target': 'speed_steps_s: 300\ntargets_mm: [50]\n'}, ()),          # max_speed 초과
            ({'target': 'speed_steps_s: 0\ntargets_mm: [50]\n'}, ()),
            ({'coordinate': COORDINATE.replace('  travel_mm: 530\n', '')}, ()),
            ({}, ('--targets', 'nan')),
        ]
        for files, extra in cases:
            with self.subTest(files=files, extra=extra):
                self.files.write(STEPPER, HOMING, TARGET, COORDINATE)
                self.files.write(**files)
                with self.assertRaises(SystemExit):
                    self.parse('--move', *extra)

    def test_move_options_require_move(self):
        for extra in (('--wait',), ('--speed', '100'), ('--targets', '10')):
            with self.subTest(extra=extra), self.assertRaises(SystemExit):
                self.parse(*extra)


class TargetRunnerTests(unittest.TestCase):
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

    def run_args(self, *extra, sleep=None, wait=None):
        args = runner.parse_args(self.files.argv(*extra))
        return runner.run(args, self.pins, gpio=self.gpio, hardware_factory=self.factory,
                          sleep=sleep or Mock(), clock=self.tick, wait=wait or Mock(return_value=''))

    def test_preflight_never_steps(self):
        self.assertEqual(self.run_args(), 0)
        self.assertEqual(self.rail.steps, 0)
        self.assertTrue(self.rail.cleaned)
        self.gpio.cleanup.assert_called_once_with((5, 6))
        self.assertIn('사전 확인 완료', self.output.getvalue())

    def test_homing_then_targets_in_order(self):
        self.assertEqual(self.run_args('--move'), 0)
        text = self.output.getvalue()
        self.assertIn('Homing: DONE', text)
        self.assertIn('[1] 목표 50.0mm → ARRIVED, 추정 위치 50.0mm', text)
        self.assertIn('[2] 목표 10.5mm → ARRIVED, 추정 위치 10.5mm', text)
        self.assertIn('[3] 목표 0.0mm → ARRIVED, 추정 위치 0.0mm', text)
        self.assertIn('결과 요약', text)
        self.assertEqual(self.rail.position, HOME)
        self.assertEqual(self.factory.call_args.kwargs['positive_dir_level'], 1)
        self.assertTrue(self.rail.cleaned)
        self.gpio.cleanup.assert_called_once_with((5, 6))

    def test_wait_prompts_each_target_and_quit(self):
        wait = Mock(side_effect=['', 'q'])
        self.assertEqual(self.run_args('--move', '--wait', wait=wait), 0)
        self.assertEqual(wait.call_count, 2)
        self.assertIn('[1] 목표 50.0mm → ARRIVED', self.output.getvalue())
        self.assertNotIn('[2] 목표', self.output.getvalue())
        self.assertIn('사용자 종료', self.output.getvalue())
        self.assertEqual(self.rail.position, HOME + 4000)

    def test_limit_during_target_move_fails(self):
        self.rail.right_at = HOME + 2000                       # 실제 레일이 25mm 지점에서 끝나는 상황
        self.assertEqual(self.run_args('--move'), 1)
        text = self.output.getvalue()
        self.assertIn('[1] 목표 50.0mm → FAILED', text)
        self.assertNotIn('[2] 목표', text)
        self.assertEqual(self.rail.position, HOME + 2000)
        self.assertTrue(self.rail.cleaned)

    def test_homing_failure_skips_targets(self):
        self.rail.left_works = False
        self.assertEqual(self.run_args('--move', '--targets', '10'), 1)
        self.assertIn('Homing: FAILED', self.output.getvalue())
        self.assertNotIn('목표 10.0mm', self.output.getvalue())

    def test_both_pressed_rejects_before_motor_output(self):
        self.rail.right_at = 0
        self.rail.position = 0
        self.assertEqual(self.run_args('--move'), 1)
        self.factory.assert_not_called()
        self.assertEqual(self.rail.steps, 0)

    def test_ctrl_c_during_target_move(self):
        def sleep(_):
            # Homing이 끝난 뒤 목표 이동 중 100스텝 지점에서 중단
            if 'Homing: DONE' in self.output.getvalue() and self.rail.position > HOME + 100:
                raise KeyboardInterrupt

        self.assertEqual(self.run_args('--move', sleep=sleep), 130)
        steps = self.rail.steps
        self.assertIn('STOP: 사용자 중단', self.output.getvalue())
        self.assertNotIn('[1] 목표', self.output.getvalue())
        self.assertLess(self.rail.position, HOME + 110)
        self.assertEqual(self.rail.steps, steps)
        self.assertTrue(self.rail.cleaned)
        self.gpio.cleanup.assert_called_once()

    def test_cleanup_failure_sets_exit_code(self):
        self.rail.cleanup = Mock(side_effect=OSError('cleanup failed'))
        self.assertEqual(self.run_args('--move', '--targets', '1'), 1)
        self.gpio.cleanup.assert_called_once()

    def test_missing_enable_rejected_for_move(self):
        self.pins['stepper']['enable'] = None
        with self.assertRaises(ValueError):
            self.run_args('--move')
        self.gpio.setup.assert_not_called()


if __name__ == '__main__':
    unittest.main()
