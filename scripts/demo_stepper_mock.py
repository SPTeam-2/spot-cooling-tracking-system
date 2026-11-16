"""
[파일(내부 구현 코드) 목적]
실제 GPIO 없이 가상 캐리지로 모터·리미트 연결 동작을 보여주는 PC 전용 데모
[코드 내용]
Rail: 가상 위치·방향·리미트 입력·STEP 동작 제공
정상 이동, 오른쪽 리미트 정지, 반대 방향 후퇴, STOP 결과와 이론 거리 출력
[플로우 간단 정리]
가상 레일·시계 생성 → 정상 이동 → 리미트 접촉 정지 → 반대 방향 후퇴 → STOP 확인

pigpio를 불러오거나 실제 GPIO를 구동하지 않음 / 레일 범위·사양은 모의 예시값
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.carriage.drive_geometry import DriveGeometry
from src.carriage.function_limit_switch import LimitSwitchFunctions
from src.carriage.function_stepper import StepperFunctions, StepperReason


class Rail:
    position = 3
    direction = 1
    def read_levels(self):
        return int(self.position > 0), int(self.position < 8)
    def check_connection(self): pass
    def set_direction(self, direction): self.direction = direction
    def enable(self): pass
    def disable(self): pass
    def step_once(self): self.position += self.direction


def main():
    rail = Rail()
    now = [0.0]
    geometry = DriveGeometry(2,20,200,16)  # 예시 사양이며 실장비에서 검증한 값이 아님
    motor = StepperFunctions(rail, LimitSwitchFunctions(rail),
                            max_speed_steps_s=100, clock=lambda:now[0])
    for name, direction, steps in [('normal',1,2),('right limit',1,20),('retreat',-1,2)]:
        motor.start_move(direction,steps,speed_steps_s=10)
        for _ in range(100):
            now[0] += 0.11
            result = motor.update()
            if result.reason != StepperReason.RUNNING:
                break
        else:
            raise RuntimeError('demo iteration bound exceeded')
        print(name, result.reason.value, result.completed_steps,
              'estimated mm:',geometry.steps_to_mm(direction*result.completed_steps))
    motor.start_move(-1,3,speed_steps_s=10)
    print('STOP:',motor.update(stop_requested=True).reason.value)


if __name__ == '__main__':
    main()
