r"""
perception 처리 속도(fps) 확인
python scripts\fps_check.py --source data\epfl_6p.avi --frames 300
"""
import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.vision.perception import Perception  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="data/epfl_6p.avi")
    p.add_argument("--frames", type=int, default=300)
    a = p.parse_args()

    perception = Perception.from_config()
    cap = cv2.VideoCapture(a.source)
    n, t0 = 0, None
    while n < a.frames:
        ok, frame = cap.read()
        if not ok:
            break
        perception.update(frame)
        n += 1
        if n == 10:                     # 처음 10프레임(모델 로딩)은 제외
            t0 = time.time()
    if t0 and n > 10:
        print(f"{(n - 10) / (time.time() - t0):.1f} fps ({n - 10}프레임)")


if __name__ == "__main__":
    main()