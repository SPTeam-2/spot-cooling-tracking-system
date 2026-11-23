"""Pi DIR 핀 출력 점검 (루프백, 멀티미터 없이).

목적: Pi의 DIR 출력 핀(BCM 21, 물리 40번)이 실제로 0/1을 내보내는지 확인한다.
      hw/bash/pin_dir_switching_test.bash와 같은 점검의 Python 버전.
내용: 점퍼로 물리 40번 ↔ 물리 37번(BCM 26, 미사용 핀)을 직접 연결하고,
      DIR에 쓴 값을 37번 입력으로 읽어 비교한다.
흐름: DIR 출력·읽기 핀 설정 → 0, 1을 번갈아 출력 → 읽은 값 비교 → 결과 출력.

주의: 점검 전에 브레드보드로 가는 DIR 선은 Pi에서 빼고, 끝난 뒤 40↔37 점퍼는 반드시 제거.

예)  python3 hw/py/dir_readback_check.py
"""

from __future__ import annotations

import time

from _hw_common import connect, load_pins

READ_PIN = 26  # 물리 37번, pins.yaml에서 쓰지 않는 핀


def main() -> None:
    dirp = load_pins()["stepper"]["dir"]
    pigpio, pi = connect()
    ok = True
    try:
        pi.set_mode(dirp, pigpio.OUTPUT)
        pi.set_mode(READ_PIN, pigpio.INPUT)
        pi.set_pull_up_down(READ_PIN, pigpio.PUD_OFF)
        for v in (0, 1, 0, 1):
            pi.write(dirp, v)
            time.sleep(0.1)
            got = pi.read(READ_PIN)
            ok &= got == v
            print(f"DIR에 {v} 출력 → 37번에서 읽은 값: {got}")
    finally:
        pi.write(dirp, 0)
        pi.stop()
    print("결과: DIR 출력 정상" if ok else "결과: 읽은 값이 다름 → 점퍼·핀 위치·DIR 출력 확인")


if __name__ == "__main__":
    main()
