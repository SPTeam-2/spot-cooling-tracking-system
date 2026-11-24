"""스테퍼 왕복 반복 점검 (0 방향 ↔ 1 방향을 계속 번갈아).

목적: DIR 전환이 정상인지, 양쪽 방향 모두 매끄럽게 움직이는지 확인한다.
      2026-11-23 현장 점검에서 양방향 모두 잘 움직인 코드를 옵션으로 정리한 것.
내용: pigpio PWM 주파수로 단계 가속 → 정속 RUN_S초 → 단계 감속 → 잠깐 정지 → 방향 반전을 반복.
흐름: 핀 출력 설정 → EN 활성화 → [DIR 설정 → 가속 → 정속 → 감속 → 반전] 반복 → Ctrl+C 시 감속 후 EN 해제.

주의: 이동량은 시간 기반 근사값. 리미트 미연결 시 캐리지를 가운데에 두고 왕복 거리가 레일 안에 들어오게 설정.

예)  python3 hw/py/stepper_shuttle.py
     python3 hw/py/stepper_shuttle.py --speed 10 --run-s 3 --cycles 4
"""

from __future__ import annotations

import argparse
import time

from _hw_common import connect, load_pins, ramp_steps, steps_per_mm


def main() -> None:
    ap = argparse.ArgumentParser(description="스테퍼 왕복 반복 점검")
    ap.add_argument("--speed", type=float, default=6.25, help="정속 속도 mm/s (기본 6.25)")
    ap.add_argument("--micro", type=int, default=8, help="A4988 마이크로스텝 (현재 배선 1/8 → 8)")
    ap.add_argument("--dir-first", type=int, choices=(0, 1), default=0)
    ap.add_argument("--run-s", type=float, default=4, help="한 방향 정속 시간(초)")
    ap.add_argument("--ramp-s", type=float, default=0.3, help="가속·감속 단계 하나당 시간(초)")
    ap.add_argument("--turn-pause", type=float, default=0.2, help="방향 바꿀 때 멈춤 시간(초)")
    ap.add_argument("--cycles", type=int, default=0, help="편도 횟수. 0이면 Ctrl+C까지")
    args = ap.parse_args()

    pins = load_pins()["stepper"]
    step, dirp, en = pins["step"], pins["dir"], pins["enable"]
    pigpio, pi = connect()

    spmm = steps_per_mm(args.micro)
    top = int(round(args.speed * spmm))
    ramp = ramp_steps(top)

    def pwm(freq: int) -> int:
        real = pi.set_PWM_frequency(step, freq)
        pi.set_PWM_dutycycle(step, 128)
        return real

    def ramp_down() -> None:
        for f in reversed(ramp):
            pwm(f)
            time.sleep(args.ramp_s)
        pi.set_PWM_dutycycle(step, 0)
        pi.write(step, 0)

    try:
        for p in (step, dirp, en):
            pi.set_mode(p, pigpio.OUTPUT)
        pi.write(step, 0)
        pi.write(en, 0)
        time.sleep(0.05)
        direction = args.dir_first
        done = 0
        while args.cycles == 0 or done < args.cycles:
            pi.write(dirp, direction)
            time.sleep(0.01)
            for f in ramp:
                pwm(f)
                time.sleep(args.ramp_s)
            real = pwm(top)
            ramp_mm = 2 * sum(f * args.ramp_s for f in ramp) / spmm
            print(f"DIR {direction} · {real / spmm:.2f}mm/s ({real}Hz) · "
                  f"편도 약 {real * args.run_s / spmm + ramp_mm:.0f}mm (계산값)")
            time.sleep(args.run_s)
            ramp_down()
            time.sleep(args.turn_pause)
            direction = 1 - direction
            done += 1
    except KeyboardInterrupt:
        print("Ctrl+C: 감속 후 정지")
        ramp_down()
    finally:
        pi.set_PWM_dutycycle(step, 0)
        pi.write(step, 0)
        pi.write(en, 1)
        pi.stop()


if __name__ == "__main__":
    main()
