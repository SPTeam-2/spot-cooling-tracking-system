"""
[파일(내부 구현 코드) 목적]
실제 Pi에서 GPIO·리미트·모터 연결을 확인하고 왼쪽(모터 쪽) 리미트 기준 Homing을 한 번 시험하는 실행부
[코드 내용]
테스트용 homing_test.yaml·stepper_test.yaml·coordinate.yaml을 읽어 Homing 설정 구성
기존 run_stepper.py의 핀 검사·GPIO 패키지 확인·시험 YAML 검증 재사용 / --home을 붙여야 실제 이동
단계 전환·최종 결과 출력, Ctrl+C 시 Homing 취소, 모든 종료 경로에서 모터·리미트 정리 시도
[플로우 간단 정리]
인자·설정 검사 → GPIO·리미트 확인 → 모터 출력 초기화 → (--home) Homing start·update 반복 → 결과 표시 → 장치 정리

종료 시 EN 해제로 토크가 풀리므로 확정한 원점은 이 실행 안에서만 유효
서버·UDP 연결과 목표 위치 이동은 포함하지 않음 / 속도·한도는 현장 확인 전 값
실행: python3 scripts/carriage/pi/run_homing.py --home
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))
from run_stepper import load_gpio, load_test_config, validate_pins
from src.carriage.drive_geometry import DriveGeometry
from src.carriage.function_carriage import CarriageFunctions
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions
from src.carriage.homing import HomingConfig, HomingProcedure, HomingStatus
from src.carriage.hw_limit_switch import LimitSwitchHardware
from src.carriage.hw_stepper import StepperHardware

HOMING_KEYS = ('fast_speed_steps_s', 'slow_speed_steps_s', 'backoff_steps', 'max_seek_steps',
               'max_reseek_steps', 'phase_timeout_s', 'settle_s')
STEPPER_KEYS = ('positive_dir_level', 'max_speed')


def load_yaml(path):
    import yaml
    try:
        with path.open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f'YAML 형식 오류({path.name}): {exc}') from exc


def load_homing_values(path):
    """테스트용 Homing YAML에서 허용된 키만 읽음. 빠진 키는 None"""
    values = load_yaml(path)
    if not isinstance(values, dict) or set(values) - set(HOMING_KEYS):
        raise ValueError('Homing YAML은 허용된 시험 조건 키만 포함해야 합니다.')
    return {key: values.get(key) for key in HOMING_KEYS}


def load_coordinate(path):
    """coordinate.yaml에서 GT2 사양과 원점 이격 거리를 읽음 (팀 설정 파일, 읽기만 함)"""
    values = load_yaml(path)
    if not isinstance(values, dict):
        raise ValueError('coordinate.yaml 형식 오류')
    try:
        offset = values['rail']['home_offset_mm']
    except (KeyError, TypeError) as exc:
        raise ValueError('coordinate.yaml에 rail.home_offset_mm이 없습니다.') from exc
    return DriveGeometry.from_config(values), offset


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config' / 'pins.yaml')
    parser.add_argument('--coordinate', type=Path, default=ROOT / 'config' / 'coordinate.yaml')
    parser.add_argument('--stepper-config', type=Path, default=ROOT / 'config' / 'stepper_test.yaml',
                        help='방향 레벨·속도 상한·통신 제한을 읽을 시험 YAML')
    parser.add_argument('--homing-config', type=Path, default=ROOT / 'config' / 'homing_test.yaml',
                        help='테스트용 Homing 속도·이격·한도 YAML')
    parser.add_argument('--home', action='store_true', help='Homing을 한 번 실행 (없으면 사전 확인만)')
    args = parser.parse_args(argv)
    try:
        stepper = load_test_config(args.stepper_config)
        homing = load_homing_values(args.homing_config)
        args.geometry, args.home_offset_mm = load_coordinate(args.coordinate)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.error(str(exc))
    args.positive_dir_level = stepper.get('positive_dir_level')
    args.max_speed = stepper.get('max_speed')
    args.io_timeout = stepper.get('io_timeout') or 0.25
    args.homing = None
    if not args.home:
        return args
    missing = [f'stepper_test.yaml {key}' for key in STEPPER_KEYS if stepper.get(key) is None]
    missing += [f'homing_test.yaml {key}' for key, value in homing.items() if value is None]
    if missing:
        parser.error('Homing 조건이 비어 있습니다: ' + ', '.join(missing))
    try:
        args.homing = HomingConfig(**homing, home_offset_mm=args.home_offset_mm)
    except (TypeError, ValueError) as exc:
        parser.error(f'Homing 조건 오류: {exc}')
    if args.homing.fast_speed_steps_s > args.max_speed:
        parser.error('fast_speed_steps_s는 stepper_test.yaml max_speed를 넘을 수 없습니다.')
    return args


def run(args, pins, *, gpio, hardware_factory=StepperHardware, sleep=time.sleep, clock=time.monotonic):
    """한 스레드에서 Homing을 진행하고 모든 종료 경로에서 각 장치 정리를 시도"""
    validate_pins(pins)
    if args.home and pins['stepper']['enable'] is None:
        raise ValueError('현장 Homing 시험은 EN 제어 배선이 필요합니다.')
    limit_hw = hardware = carriage = homing = None
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
            positive_dir_level=args.positive_dir_level if args.home else 0,
            reserved_pins=tuple(pins['limit_switch'].values()),
            command_timeout_s=args.io_timeout)
        if args.home:
            config = args.homing
            motor = StepperFunctions(hardware, limits, max_speed_steps_s=args.max_speed, clock=clock)
            carriage = CarriageFunctions(motor, limits, args.geometry)
            homing = HomingProcedure(carriage, config, clock=clock)
            print(f'Homing 설정: 빠른 {config.fast_speed_steps_s} / 느린 {config.slow_speed_steps_s} steps/s, '
                  f'이격 {config.backoff_steps}스텝, 원점 이격 {config.home_offset_mm}mm = {homing.offset_steps}스텝, '
                  f'+DIR={args.positive_dir_level} / Ctrl+C: 취소', flush=True)
            result = homing.start()
            phase = None
            while True:
                if result.phase != phase:
                    phase = result.phase
                    print(f'단계: {phase.value}', flush=True)
                if result.status != HomingStatus.RUNNING:
                    break
                sleep(0.001)
                result = homing.update()
            print(f'결과: {result.status.value}, 마지막 단계={result.phase.value}, 오류={result.error}')
            if result.status == HomingStatus.DONE:
                print(f'원점 확정: 현재 위치 {carriage.position_mm}mm '
                      '(종료 시 EN 해제로 토크가 풀리므로 이 실행 안에서만 유효)')
            exit_code = 0 if result.status == HomingStatus.DONE else 1
        else:
            print('사전 확인 완료: STEP 펄스를 출력하지 않았습니다. Homing은 --home으로 실행합니다.')
    except KeyboardInterrupt:
        if homing is not None and homing.result.status == HomingStatus.RUNNING:
            try:
                homing.update(stop_requested=True)
            except Exception as exc:
                cleanup_errors.append(str(exc))
        print('\nSTOP: 사용자 중단. Homing을 취소하며 원점은 확정하지 않습니다.')
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
