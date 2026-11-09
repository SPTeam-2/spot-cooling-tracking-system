r"""깊이 보정값 측정: 아는 거리에 서서 사람 박스 폭·높이(px)를 잼
"""

import argparse
import csv
import os
import statistics
import sys

import cv2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
from src.vision.tracker import PersonTracker

CSV = os.path.join(ROOT, "data", "depth_ref.csv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, required=True, help="카메라~사람 실제 거리(mm)")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--pose", default="stand", help="자세 메모 (stand / sit / side 등)")
    args = ap.parse_args()

    tracker = PersonTracker.from_config(os.path.join(ROOT, "config", "detector.yaml"))
    cap = cv2.VideoCapture(args.index, cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    ws, hs = [], []
    print(f"{args.dist:.0f}mm 위치에 서서 기다리세요... ({args.frames}프레임 측정, q: 중단)")
    while len(ws) < args.frames:
        ok, frame = cap.read()
        if not ok:
            break
        dets = tracker.track(frame)
        if dets:
            d = max(dets, key=lambda d: (d.bbox[2] - d.bbox[0]) * (d.bbox[3] - d.bbox[1]))  # 가장 큰 사람
            x1, y1, x2, y2 = d.bbox
            ws.append(x2 - x1); hs.append(y2 - y1)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{args.dist:.0f}mm  {len(ws)}/{args.frames}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imshow("measure_depth_ref", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
    if not ws:
        print("사람이 검출되지 않았어요.")
        return

    w, h = statistics.median(ws), statistics.median(hs)
    print(f"결과: 거리 {args.dist:.0f}mm → 박스 폭 {w:.0f}px, 높이 {h:.0f}px (중앙값, {len(ws)}프레임)")
    os.makedirs(os.path.dirname(CSV), exist_ok=True)
    new = not os.path.exists(CSV)
    with open(CSV, "a", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        if new:
            wr.writerow(["dist_mm", "width_px", "height_px", "pose"])
        wr.writerow([args.dist, w, h, args.pose])

    with open(CSV, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    print("\n지금까지 측정값 (거리×폭이 비슷하면 폭 기준 깊이 추정이 잘 맞음)")
    print(f"{'거리(mm)':>9} {'폭(px)':>7} {'높이(px)':>8} {'거리×폭':>9}  자세")
    ks = []
    for r in rows:
        d, wpx = float(r["dist_mm"]), float(r["width_px"])
        ks.append(d * wpx)
        print(f"{d:>9.0f} {wpx:>7.0f} {float(r['height_px']):>8.0f} {d * wpx:>9.0f}  {r['pose']}")
    if len(ks) >= 2:
        k = statistics.mean(ks)
        spread = (max(ks) - min(ks)) / k * 100
        print(f"\n평균 거리×폭 = {k:.0f}  (편차 {spread:.0f}%)")
        print(f"→ 선형님 depth.py에 REF_DEPTH_MM = 1000, REF_SIZE_PX = {k / 1000:.0f} 로 넣으면 됨")


if __name__ == "__main__":
    main()