"""스테퍼 연속 회전 점검 (가속 → 정속 유지 → 감속 정지).

목적: 캐리지가 지정 방향·속도로 매끄럽게 움직이는지 확인한다.
      2026-11-22 현장 점검에서 캐리지가 잘 움직인 코드를 옵션으로 정리한 것.
내용: STEP 핀에 pigpio PWM 주파수로 펄스를 내보낸다 (펄스 간격 일정).
      목표 속도보다 낮은 허용 주파수로 단계 가속 후 정속, 종료 시 같은 단계로 감속.
흐름: 핀 출력 설정 → DIR·EN 설정 → 단계 가속 → 정속 (시간 지정 또는 Ctrl+C까지) → 단계 감속 → EN 해제.

주의: 이동량은 시간 기반이라 정확한 위치 이동용이 아니다. 리미트 미연결 시 캐리지를 가운데에서 시작.

예)  python3 hw/py/stepper_spin.py --dir 0 --speed 6.25
     python3 hw/py/stepper_spin.py --dir 1 --speed 10 --seconds 5
"""

from __future__ import annotations

import argparse
import time

from _hw_common import connect, load_pins, ramp_steps, steps_per_mm, steps_per_rev


def main() -> None:
    ap = argparse.ArgumentParser(description="스테퍼 연속 회전 점검")
    ap.add_argument("--dir", type=int, choices=(0, 1), default=1, help="DIR 출력 (0 또는 1)")
    ap.add_argument("--speed", type=float, default=6.25, help="목표 속도 mm/s (기본 6.25)")
    ap.add_argument("--micro", type=int, default=8, help="A4988 마이크로스텝 (현재 배선 1/8 → 8)")
    ap.add_argument("--ramp-s", type=float, default=0.3, help="가속·감속 단계 하나당 시간(초)")
    ap.add_argument("--seconds", type=float, default=0, help="정속 유지 시간. 0이면 Ctrl+C까지")
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

    try:
        for p in (step, dirp, en):
            pi.set_mode(p, pigpio.OUTPUT)
        pi.write(step, 0)
        pi.write(dirp, args.dir)
        pi.write(en, 0)
        time.sleep(0.05)

        for f in ramp:
            pwm(f)
            time.sleep(args.ramp_s)
        real = pwm(top)
        print(f"1/{args.micro} · DIR {args.dir} · 요청 {top}Hz → 실제 {real}Hz · "
              f"약 {real / spmm:.2f}mm/s · 초당 {real / steps_per_rev(args.micro):.3f}바퀴")

        try:
            if args.seconds > 0:
                time.sleep(args.seconds)
            else:
                print("계속 회전 중 (Ctrl+C로 감속 정지)")
                while True:
                    time.sleep(1)
        except KeyboardInterrupt:
            print("Ctrl+C: 감속 후 정지")

        for f in reversed(ramp):
            pwm(f)
            time.sleep(args.ramp_s)
    except KeyboardInterrupt:
        print("중단됨")
    finally:
        pi.set_PWM_dutycycle(step, 0)
        pi.write(step, 0)
        pi.write(en, 1)
        pi.stop()


if __name__ == "__main__":
    main()
