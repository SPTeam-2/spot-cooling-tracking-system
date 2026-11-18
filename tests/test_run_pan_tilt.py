"""
[파일(내부 구현 코드) 목적]
실제 GPIO·pigpiod 없이 Pi용 팬틸트 실행부의 설정 검사·사전 확인·펄스 실측 모드·목표 각도 이동·종료 정리를 자동 검증
[코드 내용]
VirtualConnection: 핀별 마지막 펄스·출력 기록·오류 주입이 가능한 가상 pigpiod 서보 연결
PanTiltConfigTests: 빈 값, 잘못된 키·형식, 범위 밖 목표, 핀 충돌, --pulse 값, 명령줄 우선 검사
PanTiltRunnerTests: 무출력 사전 확인, --pulse 출력 후 끄기, 중립 후 목표 이동, Enter 대기·종료, 출력 오류, Ctrl+C, 정리 실패
[플로우 간단 정리]
임시 YAML 준비 → parse_args() → run() → 종료 코드·펄스 기록·출력 문구 확인

실행부 모의 검사이며 실제 서보 각도·pigpio 호환·기구 간섭 검증은 별도 필요
"""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

path = Path(__file__).resolve().parents[1] / 'scripts' / 'pan_tilt' / 'pi' / 'run_pan_tilt.py'
spec = importlib.util.spec_from_file_location('run_pan_tilt', path)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

PINS = ('fan:\n  pwm: 18\nstepper:\n  step: 20\n  dir: 21\n  enable: 16\nservo:\n  pan: 12\n  tilt: 13\n'
        'limit_switch:\n  left: 5\n  right: 6\nultrasonic:\n  trig: 23\n  echo: 24\n')
TEST = ('pan:\n  center_pulse_us: 1500\n  us_per_deg: 10\n  min_deg: -60\n  max_deg: 60\n'
        'tilt:\n  center_pulse_us: 1450\n  us_per_deg: -10\n  min_deg: -20\n  max_deg: 40\n'
        'max_speed_deg_s: 90\ntargets_deg: [[30, 10], [-20, 0], [0, 0]]\nio_timeout: 0.25\n')
NULL_TEST = ('pan:\n  center_pulse_us: null\n  us_per_deg: null\n  min_deg: null\n  max_deg: null\n'
             'tilt:\n  center_pulse_us: null\n  us_per_deg: null\n  min_deg: null\n  max_deg: null\n'
             'max_speed_deg_s: null\ntargets_deg: null\nio_timeout: 0.25\n')


class VirtualConnection:
    def __init__(self, timeout_s=0.25):
        self.timeout_s = timeout_s
        self.pulse = {}
        self.history = []
        self.fail_after = None
        self.stopped = False

    def set_servo_pulsewidth(self, pin, pulse_us):
        if self.fail_after is not None and len(self.history) >= self.fail_after:
            raise OSError('virtual servo error')
        self.pulse[pin] = pulse_us
        self.history.append((pin, pulse_us))

    def stop(self):
        self.stopped = True


class Files:
    def __init__(self, testcase):
        directory = tempfile.TemporaryDirectory()
        testcase.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.write(PINS, TEST)

    def write(self, pins=None, test=None):
        for name, text in (('pins.yaml', pins), ('test.yaml', test)):
            if text is not None:
                (self.root / name).write_text(text, encoding='utf-8')

    def argv(self, *extra):
        return ['--config', str(self.root / 'pins.yaml'), '--test-config', str(self.root / 'test.yaml'), *extra]


class PanTiltConfigTests(unittest.TestCase):
    def setUp(self):
        self.files = Files(self)

    def parse(self, *extra):
        with contextlib.redirect_stderr(io.StringIO()):
            return runner.parse_args(self.files.argv(*extra))

    def test_valid_move_config_and_cli_override(self):
        args = self.parse('--move')
        self.assertEqual(args.pins, {'pan': 12, 'tilt': 13})
        self.assertEqual(args.targets, [(30.0, 10.0), (-20.0, 0.0), (0.0, 0.0)])
        self.assertEqual(args.axes['tilt'].us_per_deg, -10)
        self.assertNotIn(12, args.reserved)
        self.assertIn(18, args.reserved)
        args = self.parse('--move', '--targets', '5', '6', '-5', '0')
        self.assertEqual(args.targets, [(5.0, 6.0), (-5.0, 0.0)])

    def test_null_values_allow_preflight_and_pulse_only(self):
        self.files.write(test=NULL_TEST)
        self.assertIsNone(self.parse().axes)
        self.assertEqual(self.parse('--pulse', 'tilt', '1600').pulse, ('tilt', 1600))
        with self.assertRaises(SystemExit):
            self.parse('--move')

    def test_shipped_test_yaml_is_null(self):
        values = runner.load_test_values(runner.ROOT / 'config' / 'pan_tilt_test.yaml')
        self.assertTrue(all(v is None for axis in ('pan', 'tilt') for v in values[axis].values()))
        self.assertIsNone(values['max_speed_deg_s'])
        self.assertIsNone(values['targets_deg'])

    def test_invalid_values(self):
        cases = [
            TEST + 'unknown: 1\n',
            TEST.replace('  max_deg: 60\n', '  max_deg: 60\n  extra: 1\n'),
            TEST.replace('targets_deg: [[30, 10], [-20, 0], [0, 0]]', 'targets_deg: [30, 10]'),
            TEST.replace('targets_deg: [[30, 10], [-20, 0], [0, 0]]', 'targets_deg: [[30, "a"]]'),
            TEST.replace('targets_deg: [[30, 10], [-20, 0], [0, 0]]', 'targets_deg: [[61, 0]]'),     # pan 범위 밖
            TEST.replace('targets_deg: [[30, 10], [-20, 0], [0, 0]]', 'targets_deg: [[0, -21]]'),    # tilt 범위 밖
            TEST.replace('  us_per_deg: 10\n', '  us_per_deg: 30\n'),                                 # 2500us 초과
            TEST.replace('max_speed_deg_s: 90', 'max_speed_deg_s: 0'),
            TEST.replace('io_timeout: 0.25', 'io_timeout: -1'),
            'pan: [1]\n',
        ]
        for text in cases:
            with self.subTest(text=text[-60:]):
                self.files.write(test=text)
                with self.assertRaises(SystemExit):
                    self.parse('--move')

    def test_pin_conflicts_and_bad_options(self):
        for pins in (PINS.replace('pan: 12', 'pan: 18'), PINS.replace('tilt: 13', 'tilt: 12'),
                     PINS.replace('servo:\n  pan: 12\n  tilt: 13\n', '')):
            with self.subTest(pins=pins[-40:]):
                self.files.write(pins=pins)
                with self.assertRaises(SystemExit):
                    self.parse()
        self.files.write(pins=PINS)
        for extra in (('--pulse', 'roll', '1500'), ('--pulse', 'pan', '400'), ('--pulse', 'pan', '1.5'),
                      ('--wait',), ('--targets', '1', '2'), ('--move', '--targets', '1'),
                      ('--move', '--pulse', 'pan', '1500')):
            with self.subTest(extra=extra), self.assertRaises(SystemExit):
                self.parse(*extra)


class PanTiltRunnerTests(unittest.TestCase):
    def setUp(self):
        self.files = Files(self)
        self.conn = VirtualConnection()
        self.now = 0.0
        self.output = io.StringIO()
        contexts = contextlib.ExitStack()
        self.addCleanup(contexts.close)
        contexts.enter_context(contextlib.redirect_stdout(self.output))
        contexts.enter_context(contextlib.redirect_stderr(self.output))

    def tick(self):
        self.now += 0.005
        return self.now

    def run_args(self, *extra, wait=None, sleep=None):
        args = runner.parse_args(self.files.argv(*extra))
        return runner.run(args, connection_factory=lambda timeout_s: self.conn, sleep=sleep or Mock(),
                          clock=self.tick, wait=wait or Mock(return_value=''))

    def test_preflight_never_outputs(self):
        self.assertEqual(self.run_args(), 0)
        self.assertEqual([p for p in self.conn.history if p[1] != 0], [])
        self.assertEqual(self.conn.pulse, {12: 0, 13: 0})       # 정리 시 끄기만
        self.assertTrue(self.conn.stopped)
        self.assertIn('사전 확인 완료', self.output.getvalue())

    def test_pulse_mode_outputs_then_turns_off(self):
        wait = Mock(return_value='')
        self.assertEqual(self.run_args('--pulse', 'tilt', '1600', wait=wait), 0)
        self.assertEqual(self.conn.history[0], (13, 1600))
        wait.assert_called_once()
        self.assertEqual(self.conn.pulse, {12: 0, 13: 0})
        self.assertIn('tilt 펄스 1600us 출력 중', self.output.getvalue())

    def test_move_centers_then_targets(self):
        self.assertEqual(self.run_args('--move'), 0)
        text = self.output.getvalue()
        self.assertEqual(self.conn.history[:2], [(12, 1500), (13, 1450)])   # 중립 먼저
        self.assertIn('[1] 목표 (30.0, 10.0) → ARRIVED, 명령 각도 (30.0, 10.0)', text)
        self.assertIn('[3] 목표 (0.0, 0.0) → ARRIVED', text)
        self.assertIn('결과 요약', text)
        self.assertIn((12, 1800), self.conn.history)
        self.assertEqual(self.conn.pulse, {12: 0, 13: 0})
        self.assertTrue(self.conn.stopped)

    def test_wait_and_quit(self):
        wait = Mock(side_effect=['', 'q'])
        self.assertEqual(self.run_args('--move', '--wait', wait=wait), 0)
        self.assertIn('[1] 목표', self.output.getvalue())
        self.assertNotIn('[2] 목표', self.output.getvalue())
        self.assertIn('사용자 종료', self.output.getvalue())

    def test_output_error_fails(self):
        self.conn.fail_after = 5
        self.assertEqual(self.run_args('--move'), 1)
        self.assertIn('FAILED', self.output.getvalue())
        self.assertTrue(self.conn.stopped)

    def test_ctrl_c_turns_off(self):
        def sleep(_):
            if len(self.conn.history) > 6:
                raise KeyboardInterrupt
        self.assertEqual(self.run_args('--move', sleep=sleep), 130)
        self.assertIn('STOP: 사용자 중단', self.output.getvalue())
        self.assertEqual(self.conn.pulse, {12: 0, 13: 0})
        self.assertTrue(self.conn.stopped)

    def test_cleanup_failure_sets_exit_code(self):
        def wait(_):
            self.conn.fail_after = len(self.conn.history)   # 끄기 명령부터 실패
            return ''
        self.assertEqual(self.run_args('--pulse', 'pan', '1500', wait=wait), 1)
        self.assertIn('출력 정리 실패', self.output.getvalue())
        self.assertTrue(self.conn.stopped)

    def test_connection_failure(self):
        args = runner.parse_args(self.files.argv('--move'))
        factory = Mock(side_effect=ConnectionRefusedError('pigpiod not running'))
        self.assertEqual(runner.run(args, connection_factory=factory, sleep=Mock(), clock=self.tick), 1)
        self.assertIn('pigpiod not running', self.output.getvalue())


if __name__ == '__main__':
    unittest.main()
