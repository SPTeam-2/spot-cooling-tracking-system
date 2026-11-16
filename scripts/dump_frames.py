r"""
특정 프레임 구간을 ID 박스와 함께 사진으로 저장 (perception 결과 그대로)
"""
import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.vision.perception import Perception  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True)
    p.add_argument("--start", type=int, required=True)
    p.add_argument("--end", type=int, required=True)
    p.add_argument("--step", type=int, default=5)
    p.add_argument("--out", default="data/dump")
    a = p.parse_args()

    os.makedirs(a.out, exist_ok=True)
    perception = Perception.from_config()
    cap = cv2.VideoCapture(a.source)
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        obs = perception.update(frame)          # 앞부분도 처리해야 추적 상태가 같아짐
        if idx < a.start or (idx - a.start) % a.step:
            continue
        img = frame.copy()
        for w in obs:
            x1, y1, x2, y2 = map(int, w.bbox)
            color = ((w.track_id * 67) % 256, (w.track_id * 131) % 256, (w.track_id * 199) % 256)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, f"{w.track_id} | {w.rail_mm:.0f}", (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.putText(img, f"frame {idx}", (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imwrite(os.path.join(a.out, f"f{idx:05d}.jpg"), img)
        print(f"frame {idx}: {[(w.track_id, w.rail_mm) for w in obs]}")
        if idx >= a.end:
            break
    cap.release()


if __name__ == "__main__":
    main()