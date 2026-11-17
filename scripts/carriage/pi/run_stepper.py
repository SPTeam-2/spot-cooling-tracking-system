"""
[파일(내부 구현 코드) 목적]
실제 Pi에서 GPIO·리미트·모터 연결을 확인하고 명시한 유한 스텝 이동을 시험하는 실행부
[코드 내용]
설정·핀 중복·GPIO 패키지 확인, 기존 리미트와 모터 기능 연결, 한 번의 이동 실행
테스트 YAML에서 이동 조건을 읽고 CLI 지정값을 우선 적용 / --move를 붙여야 실제 이동
Ctrl+C·오류·정상 종료 시 모터와 리미트의 정리를 각각 시도
[플로우 간단 정리]
인자·설정 검사 → GPIO·리미트 확인 → 모터 출력 초기화 → 선택적 이동 → 결과 표시 → 장치 정리

사전 확인도 STEP LOW·EN HIGH·DIR 초기화 수행 / 실제 이동에는 EN GPIO 배선 필요
실제 방향·속도·배선은 현장 확인 필요 / 팬·UDP 서버 통합·Homing은 포함하지 않음
"""
from __future__ import annotations

import argparse
from importlib import metadata
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions, StepperReason
from src.carriage.hw_limit_switch import LimitSwitchHardware
from src.carriage.hw_stepper import StepperHardware


def positive_float(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('유한한 양수를 입력하세요.')
    return value


def positive_int(value):
    value = int(value)
    if value <= 0:
        raise argparse.ArgumentTypeError('양의 정수를 입력하세요.')
    return value


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config' / 'pins.yaml')
    parser.add_argument('--test-config', type=Path, default=ROOT / 'config' / 'stepper_test.yaml',
                        help='현장 시험 조건 YAML. 명령줄 값이 YAML보다 우선합니다.')
    parser.add_argument('--move', action='store_true', help='유한 스텝 이동을 한 번 실행')
    parser.add_argument('--direction', type=int, choices=(-1, 1), help='-1=모터 쪽, +1=반대쪽')
    parser.add_argument('--steps', type=positive_int)
    parser.add_argument('--speed', type=positive_float, help='요청 속도(steps/s)')
    parser.add_argument('--max-speed', type=positive_float, help='현장에서 정한 속도 상한(steps/s)')
    parser.add_argument('--positive-dir-level', type=int, choices=(0, 1), help='+방향의 실제 DIR 레벨')
    parser.add_argument('--io-timeout', type=positive_float, default=None,
                        help='pigpiod 연결·명령별 제한 시간(초), 정지거리 보장값은 아님')
    args = parser.parse_args(argv)
    fields = ('direction', 'steps', 'speed', 'max_speed', 'positive_dir_level')
    # YAML에 값이 있어도 --move 없이는 이동하지 않는다. 직접 준 이동 인자는 오입력을 알린다.
    if not args.move and any(getattr(args, name) is not None for name in fields):
        parser.error('이동 옵션을 사용하려면 --move를 지정하세요.')
    try:
        profile = load_test_config(args.test_config)
        for name in (*fields, 'io_timeout'):
            if getattr(args, name) is None:
                setattr(args, name, profile.get(name))
        if args.io_timeout is None:
            args.io_timeout = 0.25
        validate_test_values({name: getattr(args, name) for name in (*fields, 'io_timeout')})
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if args.move and any(getattr(args, name) is None for name in fields):
        parser.error('이동 조건이 비어 있습니다. 테스트 YAML의 direction, steps, speed, max_speed, positive_dir_level을 입력하세요.')
    return args


def validate_test_values(values):
    """YAML의 bool·문자열·소수 스텝·비유한 값과 속도 상한 초과를 거절한다."""
    for name, value in values.items():
        if value is None:
            continue
        if name in ('direction', 'steps', 'positive_dir_level'):
            if type(value) is not int:
                raise ValueError(f'{name}: 정수가 필요합니다.')
            if name == 'direction' and value not in (-1, 1):
                raise ValueError('direction: -1 또는 1을 입력하세요.')
            if name == 'positive_dir_level' and value not in (0, 1):
                raise ValueError('positive_dir_level: 0 또는 1을 입력하세요.')
            if name == 'steps' and value <= 0:
                raise ValueError('steps: 양의 정수가 필요합니다.')
        elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name}: 유한한 양수가 필요합니다.')
    if values.get('speed') is not None and values.get('max_speed') is not None:
        if values['speed'] > values['max_speed']:
            raise ValueError('speed는 max_speed를 넘을 수 없습니다.')


def load_test_config(path):
    """시험 조건만 읽는다. 핀·벨트 사양은 기존 설정 파일에서 관리한다."""
    import yaml
    try:
        with path.open(encoding='utf-8') as stream:
            values = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f'테스트 YAML 형식 오류: {exc}') from exc
    allowed = {'direction', 'steps', 'speed', 'max_speed', 'positive_dir_level', 'io_timeout'}
    if not isinstance(values, dict) or set(values) - allowed:
        raise ValueError('테스트 YAML은 허용된 시험 조건 키만 포함해야 합니다.')
    validate_test_values(values)
    return values


def validate_pins(config):
    """모든 장치의 BCM 핀 중복과 필수 키를 출력 전에 검사한다."""
    used = {}
    def visit(node, prefix=''):
        if not isinstance(node, dict):
            raise ValueError('핀 설정은 장치별 사전이어야 합니다.')
        for key, value in node.items():
            name = f'{prefix}.{key}' if prefix else key
            if isinstance(value, dict):
                visit(value, name)
            elif value is None and name == 'stepper.enable':
                continue
            elif type(value) is not int or not 0 <= value <= 27:
                raise ValueError(f'{name}: BCM 핀은 0~27 정수여야 합니다.')
            elif value in used:
                raise ValueError(f'GPIO {value} 중복: {used[value]}, {name}')
            else:
                used[value] = name
    visit(config)
    for section, keys in (('stepper', ('step', 'dir', 'enable')), ('limit_switch', ('left', 'right'))):
        for key in keys:
            if key not in config.get(section, {}):
                raise ValueError(f'필수 핀 설정 누락: {section}.{key}')
    return config


def load_gpio():
    """실행 중인 Python의 패키지 충돌을 확인한다. 인식 실패 시 모의 입력으로 대체하지 않는다."""
    installed = []
    for package in ('RPi.GPIO', 'rpi-lgpio'):
        try:
            installed.append(f'{package}=={metadata.version(package)}')
        except metadata.PackageNotFoundError:
            pass
    if len(installed) > 1:
        raise RuntimeError('RPi.GPIO와 rpi-lgpio가 함께 설치돼 있습니다. 한 패키지만 사용하세요.')
    import RPi.GPIO as gpio
    print('Python:', sys.executable)
    print('GPIO 패키지:', ', '.join(installed) or '배포판 메타데이터 없음')
    print('GPIO 모듈:', getattr(gpio, '__file__', '확인 불가'))
    return gpio


def run(args, pins, *, gpio, hardware_factory=StepperHardware, sleep=time.sleep):
    """한 스레드에서 입력·출력을 처리하고 모든 종료 경로에서 각 장치 정리를 시도한다."""
    validate_pins(pins)
    if args.move and pins['stepper']['enable'] is None:
        raise ValueError('현장 이동 시험은 EN 제어 배선이 필요합니다.')
    limit_hw = hardware = motor = None
    cleanup_errors = []
    exit_code = 0
    try:
        limit_hw = LimitSwitchHardware(**{
            'left_pin': pins['limit_switch']['left'],
            'right_pin': pins['limit_switch']['right'], 'gpio': gpio})
        limits = LimitSwitchFunctions(limit_hw)
        state = limits.read_state()
        print(f'리미트: LEFT={int(state.left_pressed)}, RIGHT={int(state.right_pressed)}, BOTH={int(state.both_pressed)}')
        if state.both_pressed:
            raise RuntimeError('양쪽 리미트가 눌려 있습니다. 배선·입력을 확인하세요.')
        stepper = pins['stepper']
        hardware = hardware_factory(
            stepper['step'], stepper['dir'], stepper['enable'],
            positive_dir_level=args.positive_dir_level if args.move else 0,
            reserved_pins=tuple(pins['limit_switch'].values()),
            command_timeout_s=args.io_timeout)
        if args.move:
            print(f'시험 설정: {args.test_config.resolve()}, 방향={args.direction}, 스텝={args.steps}, '
                  f'속도={args.speed}, 상한={args.max_speed}, +DIR={args.positive_dir_level}, '
                  f'통신 제한={args.io_timeout}s', flush=True)
            motor = StepperFunctions(hardware, limits, max_speed_steps_s=args.max_speed)
            result = motor.start_move(args.direction, args.steps, speed_steps_s=args.speed)
            print('이동 접수:', result.reason.value, '/ Ctrl+C: STOP', flush=True)
            while result.reason == StepperReason.RUNNING:
                result = motor.update()
                if result.reason == StepperReason.RUNNING:
                    sleep(0.001)
            print(f'결과: {result.reason.value}, 확인된 펄스={result.completed_steps}/{result.requested_steps}, '
                  f'위치 불확실={result.position_uncertain}, 오류={result.error}')
            exit_code = 1 if result.reason == StepperReason.FAULT else 0
        else:
            print('사전 확인 완료: STEP 펄스를 출력하지 않았습니다.')
    except KeyboardInterrupt:
        print('\nSTOP: 사용자 중단. 잔여 이동을 취소합니다.')
        exit_code = 130
    except Exception as exc:
        print(f'오류: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        for action in (motor.stop if motor else None, hardware.cleanup if hardware else None,
                       limit_hw.cleanup if limit_hw else None):
            if action is not None:
                try:
                    action()
                except Exception as exc:
                    cleanup_errors.append(str(exc))
        if cleanup_errors:
            print('출력·핀 정리 실패: ' + '; '.join(cleanup_errors), file=sys.stderr)
            exit_code = 1
    return exit_code


def main(argv=None):
    args = parse_args(argv)
    try:
        import yaml
        with args.config.open(encoding='utf-8') as stream:
            pins = validate_pins(yaml.safe_load(stream))
        return run(args, pins, gpio=load_gpio())
    except Exception as exc:
        print(f'사전 확인 실패: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
