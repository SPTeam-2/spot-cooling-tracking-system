"""bbox 높이 vs 폭 — 깊이 추정 안정성 라이브 확인 (카메라).

이예은 리뷰 검증용: 하체가 가리면 '높이'는 작아져 깊이를 '멀다'고 오판하는지,
'폭'은 그에 비해 안정적인지를 눈으로 본다.

보는 법:
  1) 가만히 서서 h(높이)·w(폭)·depth(h) 값을 기억
  2) 같은 자리에서 앉거나 하체를 가려봄
     → h 뚝 떨어지고 depth(h)가 확 커지면(멀다 오판) = 높이 약점 확인
     → w는 거의 그대로면 = 폭이 가림에 강함

실행:  python3 scripts/check_depth_live.py --source 0   (q 종료, s 저장)
※ depth.py가 있는 브랜치(comms/pc-pi-link)에서 실행할 것.
"""

import argparse
import os
import sys
import time

import cv2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "src"))
sys.path.insert(0, os.path.join(ROOT, "src", "control"))

from src.vision.tracker import PersonTracker
from depth import bbox_to_depth_mm


def open_source(src):
    if src.isdigit():
        cap = cv2.VideoCapture(int(src), cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    else:
        cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise RuntimeError(f"영상을 열 수 없음: {src}")
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    args = ap.parse_args()

    tracker = PersonTracker.from_config(os.path.join(ROOT, "config", "detector.yaml"))
    cap = open_source(args.source)
    win = "check_depth (q: quit, s: save)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        for d in tracker.track(frame):
            x1, y1, x2, y2 = d.bbox
            h, w = y2 - y1, x2 - x1
            depth_h = bbox_to_depth_mm(h)   # 높이 기준 (비교용)
            depth_w = bbox_to_depth_mm(w)   # 폭 기준 (실제 사용)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"h={h}px  w={w}px", (x1, max(y1 - 32, 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"depth  h={depth_h:.0f}  w={depth_w:.0f}mm", (x1, max(y1 - 8, 40)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        cv2.imshow(win, frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("s"):
            os.makedirs(os.path.join(ROOT, "data"), exist_ok=True)
            out = os.path.join(ROOT, "data", f"depthchk_{int(time.time())}.jpg")
            cv2.imwrite(out, frame)
            print("저장:", out)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
