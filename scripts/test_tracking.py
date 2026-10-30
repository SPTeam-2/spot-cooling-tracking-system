r"""추적 + 이동 평균 테스트

- 사람마다 ID를 붙이고, ID별로 레일 좌표를 이동 평균으로 안정화함.
- 종료(q) 시 프레임 간 좌표 변화량 평균을 원본(raw)과 이동 평균(smooth)으로 비교 출력
  → 가만히 서 있을 때 값이 작을수록 덜 떨리는 것.
"""

import argparse
import os
import sys
import time

import cv2
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "src"))

from src.vision.tracker import PersonTracker
from src.coordinate.smoother import TrackSmoother
from src.coordinate.calibration import CoordinateTransformer

COLORS = [(0, 255, 0), (255, 128, 0), (0, 200, 255), (255, 0, 255), (0, 128, 255), (255, 255, 0)]


def color_of(tid):
    return (200, 200, 200) if tid is None else COLORS[tid % len(COLORS)]


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


def draw_rail(frame, positions, rail_range):
    # 화면 아래 레일 바 + 작업자별 레일 위치 점
    h, w = frame.shape[:2]
    x0, x1, y = 60, w - 60, h - 50
    cv2.line(frame, (x0, y), (x1, y), (220, 220, 220), 6)
    lo, hi = rail_range
    for tid, x_rail in positions:
        px = int(x0 + (x_rail - lo) / (hi - lo) * (x1 - x0))
        cv2.circle(frame, (px, y), 12, color_of(tid), -1)
        cv2.putText(frame, f"{tid}", (px - 6, y - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_of(tid), 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--window", type=int, default=5, help="이동 평균 프레임 수")
    args = ap.parse_args()

    with open(os.path.join(ROOT, "config", "coordinate.yaml"), encoding="utf-8") as f:
        rail_range = yaml.safe_load(f)["rail_range"]

    tracker = PersonTracker.from_config(os.path.join(ROOT, "config", "detector.yaml"))
    transformer = CoordinateTransformer.from_config(os.path.join(ROOT, "config", "coordinate.yaml"))
    smoother = TrackSmoother(window=args.window)
    cap = open_source(args.source)

    win = "test_tracking (q: quit, s: save)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)

    prev_raw, prev_smooth = {}, {}
    jitter_raw, jitter_smooth = [], []
    t_prev = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        dets = tracker.track(frame)
        positions, seen = [], []

        for d in dets:
            x_real, x_rail = transformer.transform(d.ref_point)
            if d.track_id is not None:
                s_real, s_rail = smoother.update(d.track_id, (x_real, x_rail))
                seen.append(d.track_id)
                if d.track_id in prev_raw:
                    jitter_raw.append(abs(x_rail - prev_raw[d.track_id]))
                    jitter_smooth.append(abs(s_rail - prev_smooth[d.track_id]))
                prev_raw[d.track_id], prev_smooth[d.track_id] = x_rail, s_rail
            else:
                s_real, s_rail = x_real, x_rail

            x1, y1, x2, y2 = d.bbox
            c = color_of(d.track_id)
            cv2.rectangle(frame, (x1, y1), (x2, y2), c, 2)
            cv2.circle(frame, d.ref_point, 6, (0, 0, 255), -1)
            cv2.putText(frame, f"ID {d.track_id} | rail {s_rail:.0f}mm (raw {x_rail:.0f})",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
            positions.append((d.track_id, s_rail))

        smoother.end_frame(seen)
        draw_rail(frame, positions, rail_range)

        now = time.time()
        fps = 1.0 / max(now - t_prev, 1e-6)
        t_prev = now
        cv2.putText(frame, f"FPS: {fps:.1f}  people: {len(dets)}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            os.makedirs(os.path.join(ROOT, "data"), exist_ok=True)
            out = os.path.join(ROOT, "data", f"tracking_{int(time.time())}.jpg")
            cv2.imwrite(out, frame)
            print("저장:", out)

    cap.release()
    cv2.destroyAllWindows()

    if jitter_raw:
        r = sum(jitter_raw) / len(jitter_raw)
        s = sum(jitter_smooth) / len(jitter_smooth)
        print(f"\n프레임 간 레일 좌표 변화량 평균: raw {r:.1f}mm → 이동평균 {s:.1f}mm "
              f"({(1 - s / r) * 100:.0f}% 감소, window={args.window}, 샘플 {len(jitter_raw)}개)")


if __name__ == "__main__":
    main()