"""스테퍼 안정 속도 점검 (PWM 방식, 단계 가속).

목적: 정해 둔 안정 속도에서 캐리지가 매끄럽게 도는지 확인한다.
      2026-11-23 현장 점검에서 1/8 스텝 기준 200Hz(5mm/s), 250Hz(6.25mm/s)에서만 안정적이었음.
내용: STEP 핀에 pigpio PWM 주파수로 펄스를 내보낸다.
      pigpio 파형(wave) 방식은 이 Pi에서 STEP 펄스가 나가지 않아 PWM 방식으로 바꿨다.
      PWM은 정해진 주파수만 쓸 수 있다 (200, 250, 320, 400Hz 등).
흐름: 출발 주파수 → 지정 주파수를 차례로 유지 → 거꾸로 단계 감속 → EN 해제.

예)  python3 hw/py/stepper_speed_sweep.py
     python3 hw/py/stepper_speed_sweep.py --hz 200 250 --seconds 3 --dir 0
"""

from __future__ import annotations

import argparse
import time

from _hw_common import PWM_ALLOWED_HZ, RAMP_START_HZ, connect, load_pins, steps_per_mm, steps_per_rev

STABLE_HZ = [200, 250]   # 현장에서 안정적으로 돈 주파수 (1/8 기준 5, 6.25mm/s)


def main() -> None:
    ap = argparse.ArgumentParser(description="스테퍼 안정 속도 점검 (PWM)")
    ap.add_argument("--hz", type=int, nargs="+", default=STABLE_HZ,
                    help=f"차례로 시험할 주파수. PWM 허용값: {PWM_ALLOWED_HZ}")
    ap.add_argument("--micro", type=int, default=8)
    ap.add_argument("--dir", type=int, choices=(0, 1), default=1)
    ap.add_argument("--seconds", type=float, default=5, help="속도마다 유지 시간(초)")
    ap.add_argument("--ramp-s", type=float, default=0.3, help="가속·감속 단계 하나당 시간(초)")
    args = ap.parse_args()

    bad = [h for h in args.hz if h not in PWM_ALLOWED_HZ]
    if bad:
        raise SystemExit(f"PWM 허용값이 아닌 주파수: {bad} → {PWM_ALLOWED_HZ} 중에서 선택")

    pins = load_pins()["stepper"]
    step, dirp, en = pins["step"], pins["dir"], pins["enable"]
    pigpio, pi = connect()
    spmm = steps_per_mm(args.micro)
    spr = steps_per_rev(args.micro)

    def pwm(hz: int) -> int:
        real = pi.set_PWM_frequency(step, hz)
        pi.set_PWM_dutycycle(step, 128)
        return real

    try:
        for p in (step, dirp, en):
            pi.set_mode(p, pigpio.OUTPUT)
        pi.write(step, 0)
        pi.write(dirp, args.dir)
        pi.write(en, 0)
        time.sleep(0.05)
        pwm(RAMP_START_HZ)
        time.sleep(args.ramp_s)
        for hz in args.hz:
            real = pwm(hz)
            print(f"1/{args.micro} · {real / spmm:.2f}mm/s · {real}Hz · 초당 {real / spr:.3f}바퀴")
            time.sleep(args.seconds)
        for hz in reversed([RAMP_START_HZ] + args.hz[:-1]):
            pwm(hz)
            time.sleep(args.ramp_s)
        print("감속 후 정지")
    except KeyboardInterrupt:
        print("중단됨")
    finally:
        pi.set_PWM_dutycycle(step, 0)
        pi.write(step, 0)
        pi.write(en, 1)
        pi.stop()


if __name__ == "__main__":
    main()
