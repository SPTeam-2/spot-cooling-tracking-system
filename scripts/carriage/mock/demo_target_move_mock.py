"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 가상 레일에서 Homing 후 목표 위치(mm) 이동 과정을 보여주는 PC 전용 데모
[코드 내용]
Rail: 가상 위치(스텝)·방향·좌우 스위치 입력·STEP 동작 제공
Homing으로 원점 확정 → 목표 여러 개로 차례대로 이동 → 목표·추정 위치·가상 물리 위치 출력
이동 도중 새 목표로 바꾸는 경우, 범위 밖 목표 거절, STOP 취소 경우 출력
[플로우 간단 정리]
가상 레일·시계 생성 → Homing → set_target() → update() 반복 → 결과 출력

pigpio·RPi.GPIO를 불러오지 않음 / 속도·한도는 모의 예시값이며 실장비 값이 아님
범위·원점 이격 거리는 coordinate.yaml rail.travel_mm·rail.home_offset_mm 값을 읽어 사용
실행: python3 scripts/carriage/mock/demo_target_move_mock.py
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import yaml
from src.carriage.drive_geometry import DriveGeometry
from src.carriage.function_carriage import CarriageFunctions
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions
from src.carriage.homing import HomingConfig, HomingProcedure, HomingStatus
from src.carriage.target_move import TargetMoveConfig, TargetMover, TargetStatus

SPEED = 400   # 모의 예시 속도 (steps/s)


class Rail:
    """위치 0 이하에서 왼쪽 눌림, 50000 이상에서 오른쪽 눌림 (모의값)"""

    def __init__(self, position):
        self.position = position
        self.direction = 1

    def read_levels(self):
        return int(self.position > 0), int(self.position < 50000)

    def check_connection(self): pass
    def set_direction(self, direction): self.direction = direction
    def enable(self): pass
    def disable(self): pass
    def step_once(self): self.position += self.direction


def main():
    with (ROOT / 'config' / 'coordinate.yaml').open(encoding='utf-8') as stream:
        coordinate = yaml.safe_load(stream)
    now = [0.0]
    clock = lambda: now[0]
    rail = Rail(3000)
    limits = LimitSwitchFunctions(rail)
    motor = StepperFunctions(rail, limits, max_speed_steps_s=SPEED, clock=clock)
    carriage = CarriageFunctions(motor, limits, DriveGeometry.from_config(coordinate))

    def tick(step):
        result = step()
        for _ in range(1000000):
            if result.status.value not in ('RUNNING', 'MOVING'):
                return result
            now[0] += 1 / SPEED + 0.0001
            result = step()
        raise RuntimeError('demo iteration bound exceeded')

    homing = HomingProcedure(carriage, HomingConfig(   # 예시값. 실장비 속도·한도는 현장 시험 후 결정
        fast_speed_steps_s=SPEED, slow_speed_steps_s=100, backoff_steps=160, max_seek_steps=60000,
        max_reseek_steps=400, home_offset_mm=coordinate['rail']['home_offset_mm'],
        phase_timeout_s=1000, settle_s=0.02), clock=clock)
    homing.start()
    result = tick(homing.update)
    home = rail.position
    print(f'[Homing] {result.status.value}, 원점 물리 위치 {home}스텝, 위치 {carriage.position_mm}mm')
    if result.status != HomingStatus.DONE:
        return

    travel = coordinate['rail']['travel_mm']
    mover = TargetMover(carriage, TargetMoveConfig(speed_steps_s=SPEED, min_mm=0, max_mm=travel))
    print(f'목표 범위 0~{travel}mm / 속도 {SPEED} steps/s\n')

    def report(label, result):
        physical = (rail.position - home) / carriage.geometry.steps_per_mm
        print(f'  {label:<24} {result.status.value:<9} 목표 {result.target_mm}mm, '
              f'추정 위치 {result.position_mm}mm, 가상 실제 위치 {physical}mm')

    print('1) 목표 여러 개로 차례대로 이동')
    for target in (100, 250.5, 30, 530, 0):
        mover.set_target(target)
        report(f'→ {target}mm', tick(mover.update))

    print('\n2) 이동 도중 새 목표로 변경 (300mm로 가다가 80mm, 원점 경유 없이 방향 전환)')
    mover.set_target(300)
    for _ in range(12000):
        now[0] += 1 / SPEED + 0.0001
        mover.update()
    report('변경 직전', mover.result)
    mover.set_target(80)
    report('→ 80mm', tick(mover.update))

    print('\n3) 범위 밖 목표')
    for bad in (-5, travel + 1):
        try:
            mover.set_target(bad)
        except ValueError as exc:
            print(f'  {bad}mm 거절: {exc}')

    print('\n4) STOP 취소 후 다시 이동')
    mover.set_target(400)
    for _ in range(4000):
        now[0] += 1 / SPEED + 0.0001
        mover.update()
    report('STOP', mover.stop())
    mover.set_target(50)
    report('→ 50mm', tick(mover.update))


if __name__ == '__main__':
    main()
