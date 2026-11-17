"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 현장 실행부의 사전 검사·이동 접수·종료 정리를 자동 검증
[코드 내용]
RunnerTests: 기본 무이동, 한 스텝 이동, 인자·핀 충돌·EN 검사, GPIO 패키지 충돌 검사
양쪽 리미트·입력 오류·초기화 실패·Ctrl+C·정리 실패 시 반환 코드와 각 장치 정리 확인
[플로우 간단 정리]
가상 GPIO·모터 생성 → 실행 인자·오류 조건 설정 → run() → 출력 횟수·종료 코드·정리 확인

현장 실행부의 모의 검사이며 실제 Pi 라이브러리 호환·기구 동작 검증은 별도 필요
"""
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
import tempfile
from unittest.mock import Mock, patch

path = Path(__file__).resolve().parents[1] / 'scripts' / 'carriage' / 'pi' / 'run_stepper.py'
spec = importlib.util.spec_from_file_location('run_stepper', path)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.pins = {'stepper': {'step': 20, 'dir': 21, 'enable': 16},
                     'limit_switch': {'left': 5, 'right': 6}, 'fan': {'pwm': 18}}
        self.gpio = Mock()
        self.gpio.input.return_value = 1
        self.hardware = Mock()
        self.factory = Mock(return_value=self.hardware)
        self.output = io.StringIO()
        contexts = contextlib.ExitStack()
        self.addCleanup(contexts.close)
        contexts.enter_context(contextlib.redirect_stdout(self.output))
        contexts.enter_context(contextlib.redirect_stderr(self.output))

    def args(self, steps='1'):
        return runner.parse_args(['--move', '--direction', '1', '--steps', steps,
                                  '--speed', '10', '--max-speed', '20', '--positive-dir-level', '1'])

    def run_args(self, args, **kwargs):
        return runner.run(args, self.pins, gpio=self.gpio, hardware_factory=self.factory, **kwargs)

    def test_preflight_never_enables_or_steps(self):
        self.assertEqual(self.run_args(runner.parse_args([])), 0)
        self.hardware.enable.assert_not_called()
        self.hardware.step_once.assert_not_called()
        self.hardware.cleanup.assert_called_once()
        self.gpio.cleanup.assert_called_once_with((5, 6))

    def test_move_exact_one_step_and_cleanup(self):
        self.assertEqual(self.run_args(self.args()), 0)
        self.hardware.step_once.assert_called_once()
        self.hardware.cleanup.assert_called_once()
        self.gpio.cleanup.assert_called_once_with((5, 6))
        self.assertIn('COMPLETED', self.output.getvalue())

    def test_input_error_before_output(self):
        self.gpio.input.side_effect = OSError('input failed')
        self.assertEqual(self.run_args(self.args()), 1)
        self.factory.assert_not_called()
        self.gpio.cleanup.assert_called_once()

    def test_both_limits_reject_before_output(self):
        self.gpio.input.return_value = 0
        self.assertEqual(self.run_args(self.args()), 1)
        self.factory.assert_not_called()

    def test_read_error_during_move_faults_without_step(self):
        self.gpio.input.side_effect = [1, 1, 1, 1, OSError('input failed')]
        self.assertEqual(self.run_args(self.args()), 1)
        self.hardware.step_once.assert_not_called()
        self.hardware.disable.assert_called_once()
        self.hardware.cleanup.assert_called_once()

    def test_motor_initialization_failure_cleans_limits(self):
        self.factory.side_effect = OSError('initialization failed')
        self.assertEqual(self.run_args(self.args()), 1)
        self.gpio.cleanup.assert_called_once()

    def test_keyboard_stop_and_cleanup_failure_still_cleans_limits(self):
        self.hardware.cleanup.side_effect = OSError('cleanup failed')
        self.assertEqual(self.run_args(self.args('2'), sleep=Mock(side_effect=KeyboardInterrupt)), 1)
        self.hardware.step_once.assert_called_once()
        self.gpio.cleanup.assert_called_once()

    def test_keyboard_stop_exit_code(self):
        self.assertEqual(self.run_args(self.args('2'), sleep=Mock(side_effect=KeyboardInterrupt)), 130)
        self.hardware.cleanup.assert_called_once()

    def test_pin_conflicts_and_missing_enable_reject_before_gpio(self):
        self.pins['fan']['pwm'] = 20
        with self.assertRaises(ValueError):
            self.run_args(self.args())
        self.gpio.setup.assert_not_called()
        self.pins['fan']['pwm'] = 18
        self.pins['stepper']['enable'] = None
        with self.assertRaises(ValueError):
            self.run_args(self.args())
        self.gpio.setup.assert_not_called()

    def test_incomplete_or_invalid_motion_arguments(self):
        for argv in (['--move'], ['--steps', '1'], ['--io-timeout', 'nan'],
                     ['--move', '--steps', '-1'], ['--speed', 'inf']):
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                runner.parse_args(argv)
        args = ['--move', '--direction', '1', '--steps', '1', '--speed', '30',
                '--max-speed', '20', '--positive-dir-level', '1']
        with self.assertRaises(SystemExit):
            runner.parse_args(args)

    def test_conflicting_gpio_packages_fail(self):
        with patch.object(runner.metadata, 'version', return_value='installed'):
            with self.assertRaisesRegex(RuntimeError, '함께 설치'):
                runner.load_gpio()


class TestConfigTests(unittest.TestCase):
    """파일 조건·CLI 우선순위·무이동 기본값과 잘못된 YAML 차단 검사."""
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.profile = Path(directory.name) / 'test.yaml'
        self.valid = 'direction: 1\nsteps: 1\nspeed: 10\nmax_speed: 20\npositive_dir_level: 0\n'
        self.profile.write_text(self.valid, encoding='utf-8')

    def parse(self, *args):
        with contextlib.redirect_stderr(io.StringIO()):
            return runner.parse_args(['--test-config', str(self.profile), *args])

    def test_yaml_move_and_cli_override(self):
        args = self.parse('--move')
        self.assertEqual((args.direction, args.steps, args.speed, args.positive_dir_level), (1, 1, 10, 0))
        self.assertEqual(self.parse('--move', '--direction', '-1').direction, -1)
        with self.assertRaises(SystemExit):
            self.parse('--move', '--speed', '21')

    def test_configured_profile_without_move_never_enables(self):
        args = self.parse()
        gpio = Mock()
        gpio.input.return_value = 1
        hardware = Mock()
        pins = {'stepper': {'step': 20, 'dir': 21, 'enable': 16}, 'limit_switch': {'left': 5, 'right': 6}}
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.run(args, pins, gpio=gpio, hardware_factory=Mock(return_value=hardware)), 0)
        hardware.enable.assert_not_called()
        hardware.step_once.assert_not_called()

    def test_null_profile_allows_preflight_only(self):
        self.profile.write_text('direction: null\n', encoding='utf-8')
        self.assertFalse(self.parse().move)
        with self.assertRaises(SystemExit):
            self.parse('--move')

    def test_invalid_yaml_and_values(self):
        cases = ['[]', 'direction: [', self.valid + 'move: true\n',
                 self.valid.replace('steps: 1', 'steps: true'),
                 self.valid.replace('steps: 1', 'steps: 1.5'),
                 self.valid.replace('speed: 10', 'speed: .nan'),
                 self.valid.replace('speed: 10', 'speed: 30'),
                 self.valid.replace('positive_dir_level: 0', 'positive_dir_level: 2')]
        for text in cases:
            self.profile.write_text(text, encoding='utf-8')
            with self.subTest(text=text), self.assertRaises(SystemExit):
                self.parse('--move')

    def test_missing_config_rejected(self):
        self.profile.unlink()
        with self.assertRaises(SystemExit):
            self.parse('--move')
