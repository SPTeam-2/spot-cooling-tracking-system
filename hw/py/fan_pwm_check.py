"""팬 PWM 점검 (D4184 MOSFET 모듈).

목적: 팬 세기가 PWM duty에 따라 바뀌는지 확인한다.
내용: pigpio PWM으로 꺼짐 → 100% → 50% → 25% → 꺼짐 순서로 출력.
      D4184 모듈은 신호 쪽 옵토커플러가 있어 1000Hz 정도의 낮은 주파수를 쓴다.
흐름: 핀 설정 → 주파수 설정 → 단계별 duty 출력·대기 → 꺼짐.

배선(2026-11-22 기준): 팬 빨강 → D4184 플러스, 팬 검정 → LOAD, 12V → 플러스·마이너스, PWM ← 브레드보드 h27.
팀 송풍 제어 로직(src/fan/fan_control.py)과는 별개인 하드웨어 점검용.

예)  python3 hw/py/fan_pwm_check.py
"""

from __future__ import annotations

import argparse
import time

from _hw_common import connect, load_pins


def main() -> None:
    ap = argparse.ArgumentParser(description="팬 PWM 점검")
    ap.add_argument("--freq", type=int, default=1000)
    ap.add_argument("--hold", type=float, default=3, help="단계마다 유지 시간(초)")
    args = ap.parse_args()

    fan = load_pins()["fan"]["pwm"]
    pigpio, pi = connect()
    try:
        pi.set_mode(fan, pigpio.OUTPUT)
        real = pi.set_PWM_frequency(fan, args.freq)
        print(f"PWM {real}Hz")
        for duty, label in ((0, "꺼짐"), (255, "100%"), (128, "50%"), (64, "25%"), (0, "꺼짐")):
            print(label)
            pi.set_PWM_dutycycle(fan, duty)
            time.sleep(args.hold)
    except KeyboardInterrupt:
        print("중단됨")
    finally:
        pi.set_PWM_dutycycle(fan, 0)
        pi.stop()


if __name__ == "__main__":
    main()
