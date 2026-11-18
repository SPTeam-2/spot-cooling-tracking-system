"""
[파일(내부 구현 코드) 목적]
실제 Pi에서 Homing으로 원점을 잡은 뒤 목표 위치(mm) 여러 곳으로 차례대로 이동시켜 보는 현장 시험 실행부
[코드 내용]
테스트용 target_move_test.yaml·homing_test.yaml·stepper_test.yaml·coordinate.yaml을 읽어 시험 조건 구성
run_stepper.py의 핀 검사·GPIO 패키지 확인·시험 YAML 검증, run_homing.py의 Homing YAML 읽기 재사용
--move를 붙여야 실제 이동 / --wait면 목표마다 Enter를 기다려 자로 위치를 잴 시간 확보
목표별 결과·추정 위치 출력, Ctrl+C 시 이동 취소, 모든 종료 경로에서 모터·리미트 정리 시도
[플로우 간단 정리]
인자·설정 검사 → GPIO·리미트 확인 → 모터 출력 초기화 → (--move) Homing → 목표마다 set_target·update 반복 → 결과 표시 → 장치 정리

종료 시 EN 해제로 토크가 풀리므로 원점·위치는 이 실행 안에서만 유효
서버·UDP 연결은 포함하지 않음 / 속도·목표는 현장 확인 전 값 / 추정 위치는 출력 펄스 기반이며 실측값 아님
실행: python3 scripts/carriage/pi/run_target_move.py --move --wait
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))
from run_stepper import load_gpio, load_test_config, validate_pins
from run_homing import load_homing_values, load_yaml
from src.carriage.drive_geometry import DriveGeometry
from src.carriage.function_carriage import CarriageFunctions
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions
from src.carriage.homing import HomingConfig, HomingProcedure, HomingStatus
from src.carriage.hw_limit_switch import LimitSwitchHardware
from src.carriage.hw_stepper import StepperHardware
from src.carriage.target_move import TargetMoveConfig, TargetMover, TargetStatus

TARGET_KEYS = ('speed_steps_s', 'targets_mm')
STEPPER_KEYS = ('positive_dir_level', 'max_speed')


def load_target_values(path):
    """테스트용 목표 이동 YAML에서 허용된 키만 읽음. 빠진 키는 None"""
    values = load_yaml(path)
    if not isinstance(values, dict) or set(values) - set(TARGET_KEYS):
        raise ValueError('목표 이동 YAML은 허용된 시험 조건 키만 포함해야 합니다.')
    return {key: values.get(key) for key in TARGET_KEYS}


def load_rail(path):
    """coordinate.yaml에서 GT2 사양·원점 이격 거리·이동 범위를 읽음 (팀 설정 파일, 읽기만 함)"""
    values = load_yaml(path)
    try:
        rail = values['rail']
        offset, travel = rail['home_offset_mm'], rail['travel_mm']
    except (KeyError, TypeError) as exc:
        raise ValueError('coordinate.yaml에 rail.home_offset_mm·rail.travel_mm이 필요합니다.') from exc
    return DriveGeometry.from_config(values), offset, travel


def valid_targets(targets):
    return (isinstance(targets, list) and len(targets) > 0
            and all(type(t) in (int, float) and math.isfinite(t) for t in targets))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config' / 'pins.yaml')
    parser.add_argument('--coordinate', type=Path, default=ROOT / 'config' / 'coordinate.yaml')
    parser.add_argument('--stepper-config', type=Path, default=ROOT / 'config' / 'stepper_test.yaml',
                        help='방향 레벨·속도 상한·통신 제한을 읽을 시험 YAML')
    parser.add_argument('--homing-config', type=Path, default=ROOT / 'config' / 'homing_test.yaml',
                        help='테스트용 Homing 속도·이격·한도 YAML')
    parser.add_argument('--target-config', type=Path, default=ROOT / 'config' / 'target_move_test.yaml',
                        help='테스트용 목표 이동 속도·목표 목록 YAML')
    parser.add_argument('--move', action='store_true', help='Homing 후 목표 이동 실행 (없으면 사전 확인만)')
    parser.add_argument('--speed', type=float, help='목표 이동 속도(steps/s), YAML보다 우선')
    parser.add_argument('--targets', type=float, nargs='+', help='목표 목록(mm), YAML보다 우선')
    parser.add_argument('--wait', action='store_true', help='목표마다 Enter를 기다림 (자로 위치 측정용)')
    args = parser.parse_args(argv)
    if not args.move and (args.speed is not None or args.targets or args.wait):
        parser.error('이동 옵션을 사용하려면 --move를 지정하세요.')
    try:
        stepper = load_test_config(args.stepper_config)
        homing = load_homing_values(args.homing_config)
        target = load_target_values(args.target_config)
        args.geometry, args.home_offset_mm, args.travel_mm = load_rail(args.coordinate)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.error(str(exc))
    args.positive_dir_level = stepper.get('positive_dir_level')
    args.max_speed = stepper.get('max_speed')
    args.io_timeout = stepper.get('io_timeout') or 0.25
    args.homing = args.target = None
    if not args.move:
        return args
    speed = args.speed if args.speed is not None else target['speed_steps_s']
    targets = args.targets if args.targets else target['targets_mm']
    missing = [f'stepper_test.yaml {key}' for key in STEPPER_KEYS if stepper.get(key) is None]
    missing += [f'homing_test.yaml {key}' for key, value in homing.items() if value is None]
    missing += [f'target_move_test.yaml {key}' for key, value in (('speed_steps_s', speed), ('targets_mm', targets))
                if value is None]
    if missing:
        parser.error('이동 조건이 비어 있습니다: ' + ', '.join(missing))
    if not valid_targets(targets):
        parser.error('targets_mm은 숫자 목록이어야 합니다.')
    try:
        args.homing = HomingConfig(**homing, home_offset_mm=args.home_offset_mm)
        args.target = TargetMoveConfig(speed_steps_s=speed, min_mm=0, max_mm=args.travel_mm)
    except (TypeError, ValueError) as exc:
        parser.error(f'이동 조건 오류: {exc}')
    if max(args.homing.fast_speed_steps_s, args.target.speed_steps_s) > args.max_speed:
        parser.error('Homing fast 속도와 목표 이동 속도는 stepper_test.yaml max_speed를 넘을 수 없습니다.')
    outside = [t for t in targets if not 0 <= t <= args.travel_mm]
    if outside:
        parser.error(f'범위(0~{args.travel_mm}mm) 밖 목표: {outside}')
    args.targets = [float(t) for t in targets]
    return args


def drive(step, sleep):
    """한 동작이 끝날 때까지 update 반복"""
    result = step()
    while result.status in (HomingStatus.RUNNING, TargetStatus.MOVING):
        sleep(0.001)
        result = step()
    return result


def run(args, pins, *, gpio, hardware_factory=StepperHardware, sleep=time.sleep, clock=time.monotonic,
        wait=input):
    """한 스레드에서 Homing·목표 이동을 진행하고 모든 종료 경로에서 각 장치 정리를 시도"""
    validate_pins(pins)
    if args.move and pins['stepper']['enable'] is None:
        raise ValueError('현장 목표 이동 시험은 EN 제어 배선이 필요합니다.')
    limit_hw = hardware = carriage = active = None
    cleanup_errors = []
    exit_code = 0
    try:
        limit_hw = LimitSwitchHardware(pins['limit_switch']['left'], pins['limit_switch']['right'], gpio=gpio)
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
        if not args.move:
            print('사전 확인 완료: STEP 펄스를 출력하지 않았습니다. 이동은 --move로 실행합니다.')
        else:
            motor = StepperFunctions(hardware, limits, max_speed_steps_s=args.max_speed, clock=clock)
            carriage = CarriageFunctions(motor, limits, args.geometry)
            homing = HomingProcedure(carriage, args.homing, clock=clock)
            print(f'목표 이동 설정: 속도 {args.target.speed_steps_s} steps/s, 범위 0~{args.travel_mm}mm, '
                  f'목표 {args.targets}, +DIR={args.positive_dir_level} / Ctrl+C: 취소', flush=True)
            active = homing
            homing.start()
            result = drive(homing.update, sleep)
            print(f'Homing: {result.status.value}, 마지막 단계={result.phase.value}, 오류={result.error}', flush=True)
            if result.status != HomingStatus.DONE:
                exit_code = 1
            else:
                mover = TargetMover(carriage, args.target)
                active = mover
                rows = []
                for index, target in enumerate(args.targets, 1):
                    if args.wait and wait(f'[{index}/{len(args.targets)}] Enter: {target}mm로 이동 / q: 종료 ').strip() == 'q':
                        print('사용자 종료')
                        break
                    mover.set_target(target)
                    moved = drive(mover.update, sleep)
                    print(f'[{index}] 목표 {target}mm → {moved.status.value}, 추정 위치 {moved.position_mm}mm, '
                          f'오류={moved.error}', flush=True)
                    rows.append((index, target, moved.status.value, moved.position_mm))
                    if moved.status != TargetStatus.ARRIVED:
                        exit_code = 1
                        break
                print('\n결과 요약 (실측 위치는 자로 재서 직접 기록)')
                print('회차 | 목표(mm) | 결과 | 추정 위치(mm)')
                for row in rows:
                    print(' | '.join(str(value) for value in row))
            active = None
    except KeyboardInterrupt:
        if active is not None:
            try:
                active.update(stop_requested=True)
            except Exception as exc:
                cleanup_errors.append(str(exc))
        print('\nSTOP: 사용자 중단. 이동을 취소합니다.')
        exit_code = 130
    except Exception as exc:
        print(f'오류: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        for action in (carriage.stop if carriage else None, hardware.cleanup if hardware else None,
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
        with args.config.open(encoding='utf-8') as stream:
            import yaml
            pins = validate_pins(yaml.safe_load(stream))
        return run(args, pins, gpio=load_gpio())
    except Exception as exc:
        print(f'사전 확인 실패: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
