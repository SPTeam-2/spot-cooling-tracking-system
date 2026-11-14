# [파일(내부 구현 코드) 목적]
# 실제 Pi에서 스위치를 눌러 좌우 입력을 터미널로 확인하는 실행 스크립트
# [코드 내용]
# pins.yaml 로드·HW/function 연결·시작 상태 출력·30ms 유지된 변화 출력·종료 정리
# 30ms 확인은 출력용 / 모터 정지 판단에 사용 금지 / 모터 구동 없음
# [플로우 간단 정리]
# 설정 읽기 → 객체 생성 → 상태 반복 읽기·변화 출력 → Ctrl+C 또는 오류 시 핀 정리
"""입력 확인 전용 / 저장소 루트에서 python3 scripts/test_limit_switch.py 실행

시작 상태와 30ms 이상 유지된 변화 출력 / 모터 구동 없음
RPi.GPIO 기반 실행 / 실제 Pi 라이브러리 호환은 별도 확인 필요
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.carriage.hw_limit_switch import LimitSwitchHardware
from src.carriage.function_limit_switch import LimitSwitchFunctions


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "pins.yaml")
    args = parser.parse_args()
    with args.config.open(encoding="utf-8") as f:
        pins = yaml.safe_load(f)["limit_switch"]

    hardware = LimitSwitchHardware(pins["left"], pins["right"])
    switches = LimitSwitchFunctions(hardware)
    try:
        print("눌림=1, 안 눌림=0 / 종료: Ctrl+C", flush=True)
        previous = switches.read_state()
        print(f"LEFT={int(previous.left_pressed)}, RIGHT={int(previous.right_pressed)}, "
              f"BOTH={int(previous.both_pressed)}", flush=True)
        candidate = previous
        changed_at = time.monotonic()
        while True:
            current = switches.read_state()
            now = time.monotonic()
            if current != candidate:
                candidate = current
                changed_at = now
            if candidate != previous and now - changed_at >= 0.03:
                print(f"LEFT={int(candidate.left_pressed)}, RIGHT={int(candidate.right_pressed)}, "
                      f"BOTH={int(candidate.both_pressed)}", flush=True)
                previous = candidate
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        hardware.cleanup()


if __name__ == "__main__":
    main()
