r"""Perception 출력 확인 (통합부에 넘길 형식 그대로 출력)
"""

import argparse
import os
import sys

import cv2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
from src.vision.perception import Perception


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    args = ap.parse_args()

    perception = Perception.from_config(os.path.join(ROOT, "config", "detector.yaml"),
                                        os.path.join(ROOT, "config", "coordinate.yaml"))
    src = args.source
    cap = cv2.VideoCapture(int(src), cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY) if src.isdigit() else cv2.VideoCapture(src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        workers = perception.update(frame)
        n += 1
        if n % 10 == 0:
            print(f"frame {n}: {[w.to_dict() for w in workers]}")
        for w in workers:
            x1, y1, x2, y2 = w.bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"ID {w.track_id} | {w.rail_mm:.0f}mm", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if w.shoulders:
                (lx, ly), (rx, ry) = w.shoulders
                cv2.line(frame, (lx, ly), (rx, ry), (0, 0, 255), 3)
        cv2.imshow("test_perception (q: quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()