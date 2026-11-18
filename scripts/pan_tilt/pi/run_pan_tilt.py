"""
[파일(내부 구현 코드) 목적]
실제 Pi에서 팬틸트 서보 펄스를 실측하고, 중립 후 목표 각도로 이동시켜 보는 현장 시험 실행부
[코드 내용]
테스트용 pan_tilt_test.yaml·pins.yaml을 읽어 축별 설정·목표 구성 (pins.yaml은 읽기만)
기본 실행: pigpiod 연결만 확인하고 펄스 출력 없음
--pulse AXIS US: 한 축에 펄스를 직접 출력하고 Enter까지 유지 → 중립·각도별 펄스 실측용
--move: 중립(0도) → 목표 [pan, tilt] 차례 이동 / --wait면 목표마다 Enter 대기
Ctrl+C 시 이동 취소, 모든 종료 경로에서 두 축 출력 끄기·연결 종료 시도
[플로우 간단 정리]
인자·설정 검사 → pigpiod 연결 → (--pulse) 펄스 출력·대기 / (--move) 중립 → 목표마다 set_target·update 반복 → 출력 끄기·정리

종료 시 펄스 0으로 서보 힘이 풀림 / 각도는 명령 각도이며 실측 아님 / 서보 전원은 강압 모듈 5V (Pi 5V 핀 직접 연결 금지)
서버·UDP 연결은 포함하지 않음 / 값은 현장 확인 전 테스트용
실행: python3 scripts/pan_tilt/pi/run_pan_tilt.py --pulse pan 1500
      python3 scripts/pan_tilt/pi/run_pan_tilt.py --move --wait
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from src.pan_tilt.function_pan_tilt import AXES, AxisConfig, PanTiltFunctions
from src.pan_tilt.hw_servo import ServoHardware
from src.pan_tilt.move_to_angle import AngleStatus, MoveToAngle, MoveToAngleConfig
from src.pan_tilt.pigpio_servo_connection import PULSE_MAX_US, PULSE_MIN_US

AXIS_KEYS = ('center_pulse_us', 'us_per_deg', 'min_deg', 'max_deg')
TOP_KEYS = ('pan', 'tilt', 'max_speed_deg_s', 'targets_deg', 'io_timeout')


def load_yaml(path):
    import yaml
    try:
        with path.open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f'YAML 형식 오류({path.name}): {exc}') from exc


def load_test_values(path):
    """테스트용 팬틸트 YAML에서 허용된 키만 읽음. 빠진 키는 None"""
    values = load_yaml(path)
    if not isinstance(values, dict) or set(values) - set(TOP_KEYS):
        raise ValueError('팬틸트 YAML은 허용된 시험 조건 키만 포함해야 합니다.')
    result = {key: values.get(key) for key in TOP_KEYS}
    for axis in AXES:
        section = result[axis] or {}
        if not isinstance(section, dict) or set(section) - set(AXIS_KEYS):
            raise ValueError(f'{axis} 항목은 {", ".join(AXIS_KEYS)}만 포함해야 합니다.')
        result[axis] = {key: section.get(key) for key in AXIS_KEYS}
    timeout = result['io_timeout']
    if timeout is None:
        result['io_timeout'] = 0.25
    elif type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('io_timeout은 유한한 양수여야 합니다.')
    return result


def servo_pins(pins):
    """pins.yaml에서 서보 핀을 읽고 다른 부품 핀과 겹치지 않는지 검사"""
    try:
        pan, tilt = pins['servo']['pan'], pins['servo']['tilt']
    except (KeyError, TypeError) as exc:
        raise ValueError('pins.yaml에 servo.pan·servo.tilt가 필요합니다.') from exc
    if any(type(pin) is not int or not 0 <= pin <= 27 for pin in (pan, tilt)) or pan == tilt:
        raise ValueError('서보 핀은 서로 다른 BCM 0~27 정수여야 합니다.')
    others = set()
    for name, section in pins.items():
        if name != 'servo' and isinstance(section, dict):
            others.update(value for value in section.values() if type(value) is int)
    clash = {pan, tilt} & others
    if clash:
        raise ValueError(f'서보 핀이 다른 부품 핀과 겹칩니다: {sorted(clash)}')
    return {'pan': pan, 'tilt': tilt}, tuple(sorted(others))


def parse_targets(values):
    if (not isinstance(values, list) or not values
            or any(not isinstance(pair, (list, tuple)) or len(pair) != 2 for pair in values)
            or any(type(v) not in (int, float) or not math.isfinite(v) for pair in values for v in pair)):
        raise ValueError('targets_deg는 [pan, tilt] 숫자 쌍의 목록이어야 합니다.')
    return [(float(pan), float(tilt)) for pan, tilt in values]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config' / 'pins.yaml')
    parser.add_argument('--test-config', type=Path, default=ROOT / 'config' / 'pan_tilt_test.yaml',
                        help='테스트용 팬틸트 YAML')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--pulse', nargs=2, metavar=('AXIS', 'US'), help='한 축에 펄스 직접 출력 (pan/tilt, 500~2500)')
    mode.add_argument('--move', action='store_true', help='중립 후 목표 각도 이동 (없으면 사전 확인만)')
    parser.add_argument('--targets', type=float, nargs='+', metavar='DEG',
                        help='목표 각도를 pan tilt 순서로 나열, YAML보다 우선 (예: 20 0 -20 10)')
    parser.add_argument('--wait', action='store_true', help='목표마다 Enter를 기다림 (각도 확인용)')
    args = parser.parse_args(argv)
    if not args.move and (args.targets or args.wait):
        parser.error('이동 옵션을 사용하려면 --move를 지정하세요.')
    try:
        values = load_test_values(args.test_config)
        args.pins, args.reserved = servo_pins(load_yaml(args.config))
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    args.io_timeout = values['io_timeout']
    args.axes = args.move_config = None
    if args.pulse:
        axis, pulse = args.pulse
        if axis not in AXES:
            parser.error("--pulse 축은 pan 또는 tilt여야 합니다.")
        try:
            pulse = int(pulse)
        except ValueError:
            parser.error('--pulse 값은 정수(us)여야 합니다.')
        if not PULSE_MIN_US <= pulse <= PULSE_MAX_US:
            parser.error('--pulse 값은 500~2500us여야 합니다.')
        args.pulse = (axis, pulse)
    if not args.move:
        return args
    targets = values['targets_deg']
    if args.targets:
        if len(args.targets) % 2:
            parser.error('--targets는 pan tilt 순서의 짝수 개 값이어야 합니다.')
        targets = [args.targets[i:i + 2] for i in range(0, len(args.targets), 2)]
    missing = [f'{axis}.{key}' for axis in AXES for key, value in values[axis].items() if value is None]
    missing += [key for key, value in (('max_speed_deg_s', values['max_speed_deg_s']), ('targets_deg', targets))
                if value is None]
    if missing:
        parser.error('이동 조건이 비어 있습니다: ' + ', '.join(missing))
    try:
        args.axes = {axis: AxisConfig(**values[axis]) for axis in AXES}
        args.move_config = MoveToAngleConfig(max_speed_deg_s=values['max_speed_deg_s'])
        args.targets = parse_targets(targets)
    except (TypeError, ValueError) as exc:
        parser.error(f'이동 조건 오류: {exc}')
    for pan, tilt in args.targets:
        for axis, deg in (('pan', pan), ('tilt', tilt)):
            config = args.axes[axis]
            if not config.min_deg <= deg <= config.max_deg:
                parser.error(f'{axis} 목표 {deg}도가 허용 범위({config.min_deg}~{config.max_deg}도) 밖입니다.')
    return args


def run(args, *, connection_factory=None, sleep=time.sleep, clock=time.monotonic, wait=input):
    """한 스레드에서 서보 출력을 진행하고 모든 종료 경로에서 출력 끄기·연결 종료 시도"""
    if connection_factory is None:
        from src.pan_tilt.pigpio_servo_connection import PigpioServoConnection
        connection_factory = PigpioServoConnection
    connection = mover = None
    servos = {}
    cleanup_errors = []
    exit_code = 0
    try:
        connection = connection_factory(timeout_s=args.io_timeout)
        servos = {axis: ServoHardware(args.pins[axis], reserved_pins=args.reserved, connection=connection)
                  for axis in AXES}
        print(f'서보 핀: pan(SG90)=BCM{args.pins["pan"]}, tilt(MG90S)=BCM{args.pins["tilt"]}, pigpiod 연결 확인')
        if args.pulse:
            axis, pulse = args.pulse
            print('주의: 허용 각도 검사 없이 펄스를 직접 출력합니다. 간섭이 보이면 바로 Ctrl+C', flush=True)
            servos[axis].set_pulse(pulse)
            print(f'{axis} 펄스 {pulse}us 출력 중', flush=True)
            wait('Enter: 출력 끄고 종료 ')
        elif not args.move:
            print('사전 확인 완료: 서보 펄스를 출력하지 않았습니다. 이동은 --move로 실행합니다.')
        else:
            pan_tilt = PanTiltFunctions(servos['pan'], servos['tilt'], args.axes['pan'], args.axes['tilt'])
            mover = MoveToAngle(pan_tilt, args.move_config, clock=clock)
            print(f'팬틸트 설정: 최대 {args.move_config.max_speed_deg_s}deg/s, '
                  f'pan {args.axes["pan"].min_deg}~{args.axes["pan"].max_deg}도, '
                  f'tilt {args.axes["tilt"].min_deg}~{args.axes["tilt"].max_deg}도 / Ctrl+C: 취소', flush=True)
            pan_tilt.center()
            print('중립(0, 0)으로 이동', flush=True)
            rows = []
            for index, (pan, tilt) in enumerate(args.targets, 1):
                if args.wait and wait(f'[{index}/{len(args.targets)}] Enter: pan {pan}도, tilt {tilt}도로 이동 / q: 종료 ').strip() == 'q':
                    print('사용자 종료')
                    break
                result = mover.set_target(pan, tilt)
                while result.status == AngleStatus.MOVING:
                    sleep(0.001)
                    result = mover.update()
                angles = ', '.join('None' if v is None else f'{v:.1f}' for v in (result.pan, result.tilt))
                print(f'[{index}] 목표 ({pan}, {tilt}) → {result.status.value}, 명령 각도 ({angles}), '
                      f'오류={result.error}', flush=True)
                rows.append((index, pan, tilt, result.status.value))
                if result.status != AngleStatus.ARRIVED:
                    exit_code = 1
                    break
            print('\n결과 요약 (실제 각도는 각도기로 재서 직접 기록)')
            print('회차 | pan 목표(도) | tilt 목표(도) | 결과')
            for row in rows:
                print(' | '.join(str(value) for value in row))
    except KeyboardInterrupt:
        if mover is not None:
            mover.stop()
        print('\nSTOP: 사용자 중단. 서보 출력을 끕니다.')
        exit_code = 130
    except Exception as exc:
        print(f'오류: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        for servo in servos.values():
            try:
                servo.cleanup()
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if connection is not None:
            try:
                connection.stop()
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if cleanup_errors:
            print('출력 정리 실패: ' + '; '.join(cleanup_errors) + ' (서보 전원을 직접 차단하세요)', file=sys.stderr)
            exit_code = 1
    return exit_code


def main(argv=None):
    args = parse_args(argv)
    try:
        return run(args)
    except Exception as exc:
        print(f'사전 확인 실패: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
