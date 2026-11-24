"""스테퍼 회전량 정확도 점검 (정해진 바퀴 수를 여러 번 반복).

목적: 보낸 스텝 수만큼 정확히 도는지 확인한다. 회전량이 매번 다르면 MS 핀 접촉 불량 의심.
      (2026-11-22: MS1 접촉 불량일 때 1/8에서 1, 1, 1, 2, 2.5바퀴로 들쭉날쭉했음)
내용: STEP을 한 펄스씩 직접 출력한다. 스텝 수는 정확하고, 속도는 Python sleep이라 약간 흔들린다.
흐름: 핀 출력 설정 → EN 활성화 → (바퀴 수 × 마이크로스텝) 펄스 출력 → 대기 → 반복 → EN 해제.

예)  python3 hw/py/stepper_rev_check.py --micro 8 --runs 5
     python3 hw/py/stepper_rev_check.py --micro 16 --sec-per-rev 8
"""

from __future__ import annotations

import argparse
import time

from _hw_common import connect, load_pins, steps_per_rev


def main() -> None:
    ap = argparse.ArgumentParser(description="스테퍼 회전량 정확도 점검")
    ap.add_argument("--micro", type=int, default=8, help="A4988 마이크로스텝 (현재 배선 1/8 → 8)")
    ap.add_argument("--revs", type=float, default=1, help="한 번에 돌릴 바퀴 수")
    ap.add_argument("--runs", type=int, default=5, help="반복 횟수")
    ap.add_argument("--sec-per-rev", type=float, default=4, help="1바퀴에 걸릴 시간(초). 클수록 느림")
    ap.add_argument("--dir", type=int, choices=(0, 1), default=0)
    ap.add_argument("--pause", type=float, default=3, help="회차 사이 확인 시간(초)")
    args = ap.parse_args()

    pins = load_pins()["stepper"]
    step, dirp, en = pins["step"], pins["dir"], pins["enable"]
    pigpio, pi = connect()

    spr = steps_per_rev(args.micro)
    steps = int(round(spr * args.revs))
    delay = args.sec_per_rev / spr / 2

    try:
        for p in (step, dirp, en):
            pi.set_mode(p, pigpio.OUTPUT)
        pi.write(step, 0)
        pi.write(dirp, args.dir)
        pi.write(en, 0)
        time.sleep(0.5)
        print(f"1/{args.micro} · {steps}스텝 = {args.revs}바퀴 · 축 표시 위치 확인")
        for r in range(1, args.runs + 1):
            t = time.time()
            for _ in range(steps):
                pi.write(step, 1)
                time.sleep(delay)
                pi.write(step, 0)
                time.sleep(delay)
            print(f"  {r}회차 완료 ({time.time() - t:.1f}초): 표시가 제자리인지 확인")
            time.sleep(args.pause)
    except KeyboardInterrupt:
        print("중단됨")
    finally:
        pi.write(step, 0)
        pi.write(en, 1)
        pi.stop()


if __name__ == "__main__":
    main()
