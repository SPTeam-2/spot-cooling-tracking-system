"""
[파일(내부 구현 코드) 목적]
실제 GPIO·pigpiod 없이 가상 서보로 팬틸트 목표 각도 이동 과정을 보여주는 PC 전용 데모
[코드 내용]
VirtualServo: 핀별 마지막 펄스를 기록하는 가상 서보 연결
중립 → 목표 각도 여러 개로 이동, 이동 중 목표 변경, 범위 밖 목표 거절, STOP, 출력 끄기 결과 출력
[플로우 간단 정리]
가상 연결·시계 생성 → 중립 → set_target() → update() 반복하며 일정 간격으로 각도·펄스 출력 → off()

pigpio를 불러오지 않음 / 중립 펄스·1도당 펄스·허용 범위·각속도는 모의 예시값이며 실측값 아님
실행: python3 scripts/pan_tilt/mock/demo_pan_tilt_mock.py
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from src.pan_tilt.function_pan_tilt import AxisConfig, PanTiltFunctions
from src.pan_tilt.hw_servo import ServoHardware
from src.pan_tilt.move_to_angle import AngleStatus, MoveToAngle, MoveToAngleConfig


class VirtualServo:
    def __init__(self):
        self.pulse = {}

    def set_servo_pulsewidth(self, pin, pulse_us):
        self.pulse[pin] = pulse_us

    def stop(self):
        pass


def main():
    conn = VirtualServo()
    now = [0.0]
    pan_tilt = PanTiltFunctions(   # 예시값. 실제 값은 run_pan_tilt.py --pulse로 실측
        ServoHardware(12, connection=conn), ServoHardware(13, connection=conn),
        AxisConfig(center_pulse_us=1500, us_per_deg=10, min_deg=-60, max_deg=60),
        AxisConfig(center_pulse_us=1450, us_per_deg=-10, min_deg=-20, max_deg=40))
    mover = MoveToAngle(pan_tilt, MoveToAngleConfig(max_speed_deg_s=90), clock=lambda: now[0])

    def deg(value):
        return 'None' if value is None else f'{value:.1f}'

    def show(label):
        result = mover.result
        print(f'  {label:<14} {result.status.value:<9} 명령 각도 pan {deg(result.pan)}, tilt {deg(result.tilt)} '
              f'| 펄스 pan {conn.pulse.get(12)}us, tilt {conn.pulse.get(13)}us')

    def run(label, ticks=100000, every=None):
        for count in range(1, ticks + 1):
            if mover.result.status != AngleStatus.MOVING:
                break
            now[0] += 0.02
            mover.update()
            if every and count % every == 0:
                show(f'  {count * 0.02:.2f}s')
        show(label)

    pan_tilt.center()
    print('예시 설정: pan -60~60도, tilt -20~40도, 최대 90deg/s, 0.02s마다 갱신\n')
    show('중립')

    print('\n1) 목표 각도로 이동 (0.2s마다 표시)')
    mover.set_target(45, 30)
    run('도착', every=10)

    print('\n2) 차례대로 이동')
    for pan, tilt in ((-30, 0), (0, -20), (60, 40)):
        mover.set_target(pan, tilt)
        run(f'→ ({pan}, {tilt})')

    print('\n3) 이동 도중 새 목표 (-60, 0)로 가다가 0.3s 후 (10, 10)')
    mover.set_target(-60, 0)
    run('변경 직전', ticks=15)
    mover.set_target(10, 10)
    run('→ (10, 10)')

    print('\n4) 범위 밖 목표')
    for pan, tilt in ((70, 0), (0, -25)):
        try:
            mover.set_target(pan, tilt)
        except ValueError as exc:
            print(f'  ({pan}, {tilt}) 거절: {exc}')

    print('\n5) STOP 후 출력 끄기')
    mover.set_target(-40, 20)
    run('STOP 직전', ticks=10)
    mover.stop()
    show('STOP')
    pan_tilt.off()
    print(f'  출력 끄기 후 펄스 {conn.pulse}, 명령 각도 pan {pan_tilt.angle("pan")} (힘이 풀려 알 수 없음)')


if __name__ == '__main__':
    main()
