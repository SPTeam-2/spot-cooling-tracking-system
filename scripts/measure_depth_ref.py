r"""깊이 보정값 측정: 아는 거리에 서서 사람 박스 폭·높이·어깨 너비(px)를 잼
"""

import argparse
import csv
import math
import os
import statistics
import sys

import cv2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
from src.vision.tracker import PersonTracker
from src.vision.perception import _shoulders

CSV = os.path.join(ROOT, "data", "depth_ref.csv")
HEADER = ["dist_mm", "width_px", "height_px", "shoulder_px", "pose"]


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

    ws, hs, ss = [], [], []
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
            sh = _shoulders(d)
            if sh:
                (lx, ly), (rx, ry) = sh
                ss.append(math.hypot(rx - lx, ry - ly))
                cv2.line(frame, (int(lx), int(ly)), (int(rx), int(ry)), (0, 0, 255), 3)
        cv2.putText(frame, f"{args.dist:.0f}mm  {len(ws)}/{args.frames}  shoulder {len(ss)}", (20, 40),
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
    s = statistics.median(ss) if ss else None
    s_txt = f"{s:.0f}px ({len(ss)}프레임)" if s else "안 보임"
    print(f"결과: 거리 {args.dist:.0f}mm → 박스 폭 {w:.0f}px, 높이 {h:.0f}px, 어깨 {s_txt} (중앙값, {len(ws)}프레임)")
    if len(ss) < len(ws) * 0.5:
        print("⚠ 어깨가 절반 이상 안 보였어요. 카메라를 정면으로 보고 다시 측정하는 걸 추천해요.")

    os.makedirs(os.path.dirname(CSV), exist_ok=True)
    if os.path.exists(CSV):
        with open(CSV, encoding="utf-8") as f:
            old_header = next(csv.reader(f), [])
        if old_header != HEADER:
            os.replace(CSV, CSV.replace(".csv", "_old.csv"))
            print("이전 형식 CSV는 depth_ref_old.csv로 옮겼어요.")
    new = not os.path.exists(CSV)
    with open(CSV, "a", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        if new:
            wr.writerow(HEADER)
        wr.writerow([args.dist, w, h, s if s else "", args.pose])

    with open(CSV, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    print("\n지금까지 측정값 (거리×크기가 비슷할수록 깊이 추정이 잘 맞음)")
    print(f"{'거리(mm)':>9} {'폭(px)':>7} {'어깨(px)':>8} {'거리×폭':>9} {'거리×어깨':>10}  자세")
    kw, ks = [], []
    for r in rows:
        d, wpx = float(r["dist_mm"]), float(r["width_px"])
        spx = float(r["shoulder_px"]) if r["shoulder_px"] else None
        kw.append(d * wpx)
        if spx:
            ks.append(d * spx)
        s_col = f"{spx:>8.0f}" if spx else f"{'-':>8}"
        k_col = f"{d * spx:>10.0f}" if spx else f"{'-':>10}"
        print(f"{d:>9.0f} {wpx:>7.0f} {s_col} {d * wpx:>9.0f} {k_col}  {r['pose']}")

    def summary(name, ks_):
        if len(ks_) < 2:
            return None
        k = statistics.mean(ks_)
        spread = (max(ks_) - min(ks_)) / k * 100
        print(f"평균 거리×{name} = {k:.0f}  (편차 {spread:.0f}%)  → 1000mm 기준 {k / 1000:.0f}px")
        return k

    print()
    k_s = summary("어깨", ks)
    k_w = summary("폭", kw)
    if k_s:
        print(f"\n→ depth.py: REF_DEPTH_MM = 1000, REF_SIZE_PX = {k_s / 1000:.0f}  (어깨 너비 기준)")
    if k_w:
        print(f"   (박스 폭으로 대체할 때 기준값: {k_w / 1000:.0f}px)")


if __name__ == "__main__":
    main()