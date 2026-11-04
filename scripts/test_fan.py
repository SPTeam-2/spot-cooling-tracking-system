"""깊이 → 송풍 세기 매핑 테스트 (하드웨어 없이).

distance_to_duty() 로직 검증 — 경계(500mm) 근처 히스테리시스 확인.

실행:  python3 scripts/test_fan.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "fan"))
from fan_control import distance_to_duty


def main() -> None:
    print("깊이(mm) → 송풍 세기(%)")
    prev = None
    for d in [300, 480, 520, 550, 1000, 1180, 1250, 2000]:
        duty = distance_to_duty(d, prev)
        label = {40: "미풍", 70: "약풍", 100: "강풍"}[duty]
        print(f"  {d:>5}mm → {duty:>3}% ({label})")
        prev = duty


if __name__ == "__main__":
    main()
