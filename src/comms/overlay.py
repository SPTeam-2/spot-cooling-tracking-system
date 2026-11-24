"""pc_sender 통합 화면: 웹캠 위에 송풍 대상·레일·상태 표시 (scripts/test_priority_live.py와 같은 모양).

빨간 굵은 박스 = 송풍 대상, 초록 = 다른 사람, 아래 막대 = 레일(사람 점·캐리지 추정 위치),
오른쪽 위 표 = ID별 누적 송풍·대기 시간, 왼쪽 위 = 현재 상태(송풍 중 / 사람 없음 → 정지).
"""

from __future__ import annotations

import cv2

RED, GREEN, BLUE, WHITE, YELLOW, GRAY = (0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 255), \
    (0, 255, 255), (160, 160, 160)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _text(img, s, org, color, scale=0.6, thick=2):
    cv2.putText(img, s, org, FONT, scale, (0, 0, 0), thick + 3)
    cv2.putText(img, s, org, FONT, scale, color, thick)


def motor_on_right(transformer, w, h):
    """캘리브레이션 결과로 화면 오른쪽 끝이 레일 0(모터 쪽)에 가까운지 판단 (막대 방향 맞추기)."""
    try:
        _, left = transformer.transform((0, h - 1))
        _, right = transformer.transform((w - 1, h - 1))
        return right < left
    except Exception:
        return False


def draw(frame, obs, integ, rail_range, duty, active, sonar_mm=None, flip=False):
    tid = integ.target.wid if active and integ.target is not None else None
    for o in obs:
        x1, y1, x2, y2 = o.bbox
        hit = o.track_id == tid
        c = RED if hit else GREEN
        cv2.rectangle(frame, (x1, y1), (x2, y2), c, 4 if hit else 2)
        label = f"ID {o.track_id} | rail {o.rail_mm:.0f}mm" + ("  << TARGET" if hit else "")
        _text(frame, label, (x1, max(y1 - 10, 20)), c)

    h, w = frame.shape[:2]
    _rail_bar(frame, obs, integ, rail_range, tid, w, h, flip)
    _table(frame, integ, tid, w)
    _status(frame, integ, duty, active, sonar_mm, len(obs))
    return frame


def _rail_bar(frame, obs, integ, rail_range, tid, w, h, flip):
    """flip=True면 레일 0(모터)을 오른쪽에 그림 — 화면 속 모터 위치와 같은 쪽."""
    lo, hi = rail_range
    x0, x1, y = 60, w - 60, h - 50

    def to_px(v):
        r = (min(max(v, lo), hi) - lo) / (hi - lo)
        return int(x1 - r * (x1 - x0)) if flip else int(x0 + r * (x1 - x0))

    cv2.line(frame, (x0, y), (x1, y), (220, 220, 220), 6)
    _text(frame, f"{lo:.0f}mm (motor)", (to_px(lo) - (110 if flip else 10), y + 32), WHITE, 0.5, 1)
    _text(frame, f"{hi:.0f}mm", (to_px(hi) - (10 if flip else 50), y + 32), WHITE, 0.5, 1)
    for o in obs:
        cv2.circle(frame, (to_px(o.rail_mm), y), 9, RED if o.track_id == tid else GREEN, -1)
    cx = to_px(integ.carriage_mm)
    cv2.rectangle(frame, (cx - 18, y - 12), (cx + 18, y + 12), BLUE, -1)
    _text(frame, f"carriage ~{integ.carriage_mm:.0f}mm", (cx - 70, y - 20), BLUE)


def _table(frame, integ, tid, w):
    rows = sorted((i, wk) for i, wk in integ.workers.items() if wk.detected)
    cv2.rectangle(frame, (w - 330, 10), (w - 10, 50 + 28 * len(rows)), (0, 0, 0), -1)
    cv2.putText(frame, "ID  cooled(s)  wait(s)", (w - 320, 38), FONT, 0.6, WHITE, 2)
    for i, (wid, wk) in enumerate(rows):
        cv2.putText(frame, f"{wid:<3} {wk.cooled_total_s:>8.1f} {wk.wait_s:>8.1f}",
                    (w - 320, 66 + 28 * i), FONT, 0.6, RED if wid == tid else WHITE, 2)


def _status(frame, integ, duty, active, sonar_mm, n_people):
    if active:
        head, color = f"BLOWING  fan {duty}%", RED
        depth = f"depth {integ.depth_mm:.0f}mm (cam {integ.cam_depth_mm:.0f}" + \
                (f", sonar {sonar_mm:.0f})" if sonar_mm is not None else ", sonar -)")
        lines = [depth, f"pan {integ.pan_deg:+.1f}  tilt {integ.tilt_deg:+.1f}",
                 f"dwell left {max(integ.dwell_left_s, 0):.1f}s"]
    else:
        head, color = ("NO PERSON -> STOP (fan off)" if n_people == 0
                       else "WAITING (lost < 2s / cooldown)"), YELLOW
        lines = [f"sonar {sonar_mm:.0f}mm" if sonar_mm is not None else "sonar -"]
    cv2.rectangle(frame, (10, 10), (470, 50 + 28 * len(lines)), (0, 0, 0), -1)
    cv2.putText(frame, head, (20, 40), FONT, 0.8, color, 2)
    for i, s in enumerate(lines):
        cv2.putText(frame, s, (20, 70 + 28 * i), FONT, 0.6, GRAY if not active else WHITE, 2)
