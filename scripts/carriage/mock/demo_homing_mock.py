"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 가상 레일에서 Homing 단계 진행과 원점 확정 과정을 보여주는 PC 전용 데모
[코드 내용]
Rail: 가상 위치(스텝)·방향·좌우 스위치 입력·STEP 동작 제공
시작 위치 3곳(중간·왼쪽 눌림·오른쪽 눌림)에서 Homing 실행, 단계 전환과 최종 원점 물리 위치 출력
Homing 이후 +10mm 이동 결과와 원점 기준 위치(mm) 출력
[플로우 간단 정리]
가상 레일·시계 생성 → Homing start() → update() 반복하며 단계 출력 → 원점 확정 → 짧은 이동 확인

pigpio·RPi.GPIO를 불러오지 않음 / 레일 범위·속도·한도는 모의 예시값이며 실장비 값이 아님
원점 이격 거리는 coordinate.yaml rail.home_offset_mm 값을 읽어 사용
실행: python3 scripts/carriage/mock/demo_homing_mock.py
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import yaml
from src.carriage.drive_geometry import DriveGeometry
from src.carriage.function_carriage import CarriageFunctions
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions, StepperReason
from src.carriage.homing import HomingConfig, HomingProcedure, HomingStatus


class Rail:
    """위치 0 이하에서 왼쪽 눌림, 5000 이상에서 오른쪽 눌림 (모의값)"""

    def __init__(self, position):
        self.position = position
        self.direction = 1

    def read_levels(self):
        return int(self.position > 0), int(self.position < 5000)

    def check_connection(self): pass
    def set_direction(self, direction): self.direction = direction
    def enable(self): pass
    def disable(self): pass
    def step_once(self): self.position += self.direction


def build(start, config, now):
    rail = Rail(start)
    clock = lambda: now[0]
    limits = LimitSwitchFunctions(rail)
    motor = StepperFunctions(rail, limits, max_speed_steps_s=100, clock=clock)
    carriage = CarriageFunctions(motor, limits, DriveGeometry.from_config(config))
    return rail, carriage, clock


def run_homing(start, coordinate, now):
    rail, carriage, clock = build(start, coordinate, now)
    homing_config = HomingConfig(   # 예시값. 실장비 속도·한도는 현장 시험 후 결정
        fast_speed_steps_s=100, slow_speed_steps_s=50, backoff_steps=160,
        max_seek_steps=6000, max_reseek_steps=400,
        home_offset_mm=coordinate['rail']['home_offset_mm'],
        phase_timeout_s=1000, settle_s=0.02)
    homing = HomingProcedure(carriage, homing_config, clock=clock)
    print(f'\n[시작 위치 {start}스텝] 원점 이격 {homing_config.home_offset_mm}mm = {homing.offset_steps}스텝')
    result = homing.start()
    phase = None
    for _ in range(200000):
        if result.phase != phase:
            phase = result.phase
            print(f'  {phase.value:<9} 가상 위치 {rail.position}')
        if result.status != HomingStatus.RUNNING:
            break
        now[0] += 0.011
        result = homing.update()
    else:
        raise RuntimeError('demo iteration bound exceeded')
    print(f'  결과 {result.status.value}, 원점 물리 위치 {rail.position}, '
          f'homed={carriage.homed}, 위치 {carriage.position_mm}mm, 오류={result.error}')
    return rail, carriage


def main():
    with (ROOT / 'config' / 'coordinate.yaml').open(encoding='utf-8') as stream:
        coordinate = yaml.safe_load(stream)
    now = [0.0]
    for start in (2000, -10, 5000):   # 중간 / 왼쪽 눌림 / 오른쪽 눌림
        rail, carriage = run_homing(start, coordinate, now)
    steps = carriage.geometry.mm_to_steps(10)
    result = carriage.start_move(1, steps, speed_steps_s=100)
    while result.reason == StepperReason.RUNNING:
        now[0] += 0.011
        result = carriage.update()
    print(f'\n원점 이후 +10mm({steps}스텝) 이동: {result.reason.value}, 현재 위치 {carriage.position_mm}mm')


if __name__ == '__main__':
    main()
