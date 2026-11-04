"""bbox 높이 → 깊이(mm) 추정 + 초음파 융합 확인 (하드웨어 없이).

실행:  python3 scripts/test_depth.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "control"))
from depth import bbox_to_depth_mm, fuse_depth_mm


def main() -> None:
    print("박스높이(px) → 깊이(mm)   [기준: 500px = 1000mm]")
    for h in [1000, 700, 500, 350, 250, 150]:
        print(f"  {h:>4}px → {bbox_to_depth_mm(h):>6.0f}mm")

    print("\n초음파 보조 융합 (카메라 480mm 추정 가정):")
    for sonar in [None, 500.0, 1200.0]:
        tag = "없음" if sonar is None else f"{sonar:.0f}mm"
        print(f"  sonar={tag:>7} → {fuse_depth_mm(480.0, sonar):>6.0f}mm")


if __name__ == "__main__":
    main()
