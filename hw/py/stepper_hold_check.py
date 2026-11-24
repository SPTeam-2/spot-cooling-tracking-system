"""A4988 홀딩 토크 점검 (EN만 켜고 유지).

목적: A4988이 두 코일 모두에 전류를 보내는지 손으로 확인한다.
      실행 중 축을 손으로 돌려 모든 위치에서 고르게 버티면 정상.
내용: STEP 펄스 없이 EN만 LOW(활성)로 유지한다.
흐름: 핀 출력 설정 → EN 활성화 → 지정 시간 대기 → EN 해제.

예)  python3 hw/py/stepper_hold_check.py --seconds 10
"""

from __future__ import annotations

import argparse
import time

from _hw_common import connect, load_pins


def main() -> None:
    ap = argparse.ArgumentParser(description="A4988 홀딩 토크 점검")
    ap.add_argument("--seconds", type=float, default=10)
    args = ap.parse_args()

    pins = load_pins()["stepper"]
    step, en = pins["step"], pins["enable"]
    pigpio, pi = connect()
    try:
        for p in (step, en):
            pi.set_mode(p, pigpio.OUTPUT)
        pi.write(step, 0)
        pi.write(en, 0)
        print(f"EN 켜짐: {args.seconds:.0f}초 동안 축을 손으로 돌려 보세요")
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        print("중단됨")
    finally:
        pi.write(en, 1)
        print("EN 꺼짐 (이제 손으로 쉽게 돌아가야 정상)")
        pi.stop()


if __name__ == "__main__":
    main()
