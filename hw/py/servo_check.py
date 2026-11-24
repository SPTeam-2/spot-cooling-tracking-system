"""팬틸트 서보 점검 (pan SG90, tilt MG90S).

목적: 서보가 움직이는지, 팬틸트 브래킷에 걸리지 않는 안전 범위가 어디까지인지 확인한다.
내용: pigpio 서보 펄스(µs)로 가운데(1500) → 아래 값 → 위 값 → 가운데를 범위별로 반복.
      범위는 좁은 것부터 넓혀 가므로, 걸리거나 소리가 나는 범위의 직전이 안전 최대값.
흐름: 대상 서보 선택 → 범위마다 1500 → lo → hi → 1500 → 신호 끄기.

예)  python3 hw/py/servo_check.py --target pan
     python3 hw/py/servo_check.py --target tilt --ranges 1300-1700 1100-1900 1000-2000
"""

from __future__ import annotations

import argparse
import time

from _hw_common import connect, load_pins


def parse_range(text: str) -> tuple[int, int]:
    lo, hi = (int(x) for x in text.split("-"))
    if not 500 <= lo < hi <= 2500:
        raise argparse.ArgumentTypeError("범위는 500~2500 사이 '낮은값-높은값' 형식")
    return lo, hi


def main() -> None:
    ap = argparse.ArgumentParser(description="팬틸트 서보 점검")
    ap.add_argument("--target", choices=("pan", "tilt", "both"), default="both")
    ap.add_argument("--ranges", type=parse_range, nargs="+",
                    default=[(1300, 1700), (1100, 1900), (1000, 2000)])
    ap.add_argument("--hold", type=float, default=1, help="위치마다 유지 시간(초)")
    args = ap.parse_args()

    servo = load_pins()["servo"]
    targets = ["pan", "tilt"] if args.target == "both" else [args.target]
    pigpio, pi = connect()
    try:
        for name in targets:
            pin = servo[name]
            for lo, hi in args.ranges:
                print(f"{name} · 범위 {lo} ~ {hi}")
                for pw in (1500, lo, hi, 1500):
                    pi.set_servo_pulsewidth(pin, pw)
                    time.sleep(args.hold)
    except KeyboardInterrupt:
        print("중단됨")
    finally:
        for name in ("pan", "tilt"):
            pi.set_servo_pulsewidth(servo[name], 0)
        pi.stop()


if __name__ == "__main__":
    main()
