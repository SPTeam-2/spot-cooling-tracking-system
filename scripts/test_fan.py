"""거리 → 송풍 세기 매핑 테스트 (하드웨어 없이)

src/fan/fan_control.py의 distance_to_duty() 로직만 검증한다.
경계(50cm) 근처에서 히스테리시스로 단계가 덜 흔들리는지 확인.

실행:  python3 scripts/test_fan.py
"""

from __future__ import annotations

import os
import sys

# 로직 모듈(src/fan/fan_control.py) import 경로 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "fan"))
from fan_control import distance_to_duty


def main() -> None:
    print("거리(cm) → 송풍 세기(%)")
    prev = None
    for d in [30, 48, 52, 55, 100, 118, 125, 200]:
        duty = distance_to_duty(d, prev)
        label = {40: "미풍", 70: "약풍", 100: "강풍"}[duty]
        print(f"  {d:>4}cm → {duty:>3}% ({label})")
        prev = duty


if __name__ == "__main__":
    main()
