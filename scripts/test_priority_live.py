r"""인식 → 우선순위 연결 데모 (가상 캐리지)

카메라/영상에서 추적한 작업자들을 통합부 우선순위 알고리즘(src/control/priority.py)에 넣고,
선정된 대상에게 가상 캐리지가 이동해 '송풍'하는 모습을 화면에 보여준다.
  - 빨간 굵은 박스 = 현재 송풍 대상
  - 오른쪽 위 표 = ID별 누적 송풍시간 / 대기시간
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
sys.path.insert(0, os.path.join(ROOT, "src", "control"))

from src.vision.tracker import PersonTracker
from src.coordinate.smoother import TrackSmoother
from src.coordinate.calibration import CoordinateTransformer
from priority import Worker, pick_target

CARRIAGE_SPEED = 200.0   # 가상 캐리지 속도 (mm/s) — 실제 값은 구동부 측정 후 수정
LOST_TIMEOUT_S = 2.0     # 이 시간 이상 안 보이면 '미검출' 처리


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

    with open(os.path.join(ROOT, "config", "coordinate.yaml"), encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    rail_lo, rail_hi = cfg["rail_range"]
    ctrl = cfg.get("control", {})
    min_dwell_s = ctrl.get("min_dwell_s", 8)
    cooldown_s = ctrl.get("cooldown_s", 5)
    pr = cfg.get("priority", {})

    tracker = PersonTracker.from_config(os.path.join(ROOT, "config", "detector.yaml"))
    transformer = CoordinateTransformer.from_config(os.path.join(ROOT, "config", "coordinate.yaml"))
    smoother = TrackSmoother(window=5)
    cap = open_source(args.source)

    win = "test_priority_live (q: quit, s: save)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)

    workers = {}        # track_id → Worker
    rail_pos = {}       # track_id → 레일 좌표(mm)
    last_seen = {}      # track_id → 마지막으로 보인 시각
    carriage = (rail_lo + rail_hi) / 2
    target = None
    dwell_left = 0.0
    t0 = t_prev = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        now = time.time()
        dt = now - t_prev
        t_prev = now
        t = now - t0

        # 1) 추적 + 좌표 변환 + 이동 평균
        dets = tracker.track(frame)
        seen = []
        for d in dets:
            if d.track_id is None:
                continue
            x_real, x_rail = transformer.transform(d.ref_point)
            _, s_rail = smoother.update(d.track_id, (x_real, x_rail))
            rail_pos[d.track_id] = s_rail
            last_seen[d.track_id] = now
            workers.setdefault(d.track_id, Worker(wid=d.track_id))
            seen.append(d.track_id)
        smoother.end_frame(seen)

        # 2) Worker 상태 갱신 (이름은 distance_cm이지만 레일 거리 mm를 넣음)
        for tid, w in workers.items():
            w.detected = (now - last_seen[tid]) < LOST_TIMEOUT_S
            w.distance_cm = abs(rail_pos[tid] - carriage)

        # 3) 대상 선정 (최소 유지시간 끝났거나 대상이 사라지면)
        if target is None or dwell_left <= 0 or not target.detected:
            if target is not None and dwell_left <= 0:
                target.cooldown_until = t + cooldown_s
            new = pick_target(list(workers.values()), t,
                              cooled_similar_s=pr.get("cooled_similar_s", 20.0),
                              wait_similar_s=pr.get("wait_similar_s", 20.0))
            if new is not target:
                target = new
                dwell_left = min_dwell_s

        # 4) 가상 캐리지 이동 + 송풍/대기 시간 누적 (도착해야 송풍으로 침)
        arrived = False
        if target is not None:
            goal = rail_pos[target.wid]
            step = CARRIAGE_SPEED * dt
            carriage = goal if abs(goal - carriage) <= step else carriage + step * (1 if goal > carriage else -1)
            arrived = abs(goal - carriage) < 20
        for w in workers.values():
            if not w.detected:
                continue
            if w is target and arrived:
                w.cooled_total_s += dt
                w.wait_s = 0
            else:
                w.wait_s += dt
        if target is not None and arrived:
            dwell_left -= dt

        # 5) 그리기
        for d in dets:
            x1, y1, x2, y2 = d.bbox
            is_target = target is not None and d.track_id == target.wid
            c = (0, 0, 255) if is_target else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), c, 4 if is_target else 2)
            rail_txt = f" | rail {rail_pos[d.track_id]:.0f}mm" if d.track_id in rail_pos else ""
            label = f"ID {d.track_id}{rail_txt}" + ("  << TARGET" if is_target else "")
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)

        h, w_img = frame.shape[:2]
        x0, x1r, y = 60, w_img - 60, h - 50
        to_px = lambda v: int(x0 + (v - rail_lo) / (rail_hi - rail_lo) * (x1r - x0))
        cv2.line(frame, (x0, y), (x1r, y), (220, 220, 220), 6)
        for tid, pos in rail_pos.items():
            if workers[tid].detected:
                cv2.circle(frame, (to_px(pos), y), 8, (0, 255, 0), -1)
        cv2.rectangle(frame, (to_px(carriage) - 18, y - 12), (to_px(carriage) + 18, y + 12), (255, 0, 0), -1)
        cv2.putText(frame, f"carriage {carriage:.0f}mm", (to_px(carriage) - 60, y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        rows = sorted((tid, wk) for tid, wk in workers.items() if wk.detected)
        cv2.rectangle(frame, (w_img - 330, 10), (w_img - 10, 50 + 28 * len(rows)), (0, 0, 0), -1)
        cv2.putText(frame, "ID  cooled(s)  wait(s)", (w_img - 320, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        for i, (tid, wk) in enumerate(rows):
            cv2.putText(frame, f"{tid:<3} {wk.cooled_total_s:>8.1f} {wk.wait_s:>8.1f}",
                        (w_img - 320, 66 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255) if wk is target else (255, 255, 255), 2)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            os.makedirs(os.path.join(ROOT, "data"), exist_ok=True)
            out = os.path.join(ROOT, "data", f"priority_{int(time.time())}.jpg")
            cv2.imwrite(out, frame)
            print("저장:", out)

    cap.release()
    cv2.destroyAllWindows()
    print("\nID별 누적 송풍시간(s):")
    for tid, wk in sorted(workers.items()):
        print(f"  ID {tid}: {wk.cooled_total_s:.1f}")


if __name__ == "__main__":
    main()