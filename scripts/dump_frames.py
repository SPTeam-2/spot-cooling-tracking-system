r"""
특정 프레임 구간을 ID 박스와 함께 보기/저장 (perception 결과 그대로)
- 사진 저장:  python scripts\dump_frames.py --source data\epfl_6p.avi --start 1660 --end 1730 --step 5
- 화면 보기:  python scripts\dump_frames.py --source data\epfl_6p.avi --start 1 --end 99999 --show --delay 60
              (스페이스: 일시정지/재생, d: 한 프레임 앞으로(일시정지 중), q: 종료, 왼쪽 위에 프레임 번호)
- 원인 로그:  python scripts\dump_frames.py --source data\epfl_6p.avi --start 2200 --end 2600 --step 0 --debug
              (start~end 사이 새 번호/재연결 이벤트만 프레임 번호와 함께 출력, --step 0 은 사진 저장 안 함)
"""
import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.vision.perception import Perception  # noqa: E402


def draw(frame, obs, idx):
    img = frame.copy()
    for w in obs:
        x1, y1, x2, y2 = map(int, w.bbox)
        color = ((w.track_id * 67) % 256, (w.track_id * 131) % 256, (w.track_id * 199) % 256)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, f"{w.track_id} | {w.rail_mm:.0f}", (x1, max(y1 - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText(img, f"frame {idx}", (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return img


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True)
    p.add_argument("--start", type=int, required=True)
    p.add_argument("--end", type=int, required=True)
    p.add_argument("--step", type=int, default=5, help="사진 저장 간격 (--show 아닐 때, 0이면 저장 안 함)")
    p.add_argument("--out", default="data/dump")
    p.add_argument("--from-frame", type=int, default=0, help="이 프레임부터 처리 시작 (0이면 처음부터)")
    p.add_argument("--debug", action="store_true", help="start~end 사이 새 번호/재연결 이벤트 출력")
    p.add_argument("--show", action="store_true", help="사진 저장 대신 화면으로 보기")
    p.add_argument("--delay", type=int, default=60, help="--show 때 프레임 간 대기(ms)")
    a = p.parse_args()

    if not a.show and a.step > 0:
        os.makedirs(a.out, exist_ok=True)
    perception = Perception.from_config()
    rl = perception.relinker
    cap = cv2.VideoCapture(a.source)
    idx = 0
    if a.from_frame > 0:                       # 앞부분 건너뛰기 (추적은 여기서부터 새로 시작)
        cap.set(cv2.CAP_PROP_POS_FRAMES, a.from_frame - 1)
        idx = a.from_frame - 1
    paused = False
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if rl is not None:
            rl.debug = a.debug and a.start <= idx <= a.end   # 구간 안에서만 이벤트 출력
            rl.tag = f"[frame {idx}]"
        obs = perception.update(frame)          # 앞부분도 처리해야 추적 상태가 같아짐
        if idx < a.start:
            continue

        if a.show:
            cv2.imshow("dump_frames (space: pause, d: next, q: quit)", draw(frame, obs, idx))
            key = cv2.waitKey(0 if paused else a.delay) & 0xFF
            if key == ord(" "):
                paused = not paused
            if key == ord("q"):
                break
        elif a.step > 0 and (idx - a.start) % a.step == 0:
            cv2.imwrite(os.path.join(a.out, f"f{idx:05d}.jpg"), draw(frame, obs, idx))
            print(f"frame {idx}: {[(w.track_id, w.rail_mm) for w in obs]}")
        if idx >= a.end:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()