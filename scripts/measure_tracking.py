r"""
추적 ID 안정성 측정
- 같은 영상에서 최종 ID 개수 / 오래 유지된 ID 개수 / FPS 를 출력
- --relink 를 붙이면 트래커 ID를 IdRelinker로 재연결한 '안정 ID' 기준으로 측정
- 사용 예:
  python scripts\measure_tracking.py --source data\epfl_6p.avi --model models\yolo11n-pose.pt --conf 0.1 --tracker config\botsort_custom.yaml
  python scripts\measure_tracking.py --source data\epfl_6p.avi --model models\yolo11n-pose.pt --conf 0.1 --tracker config\botsort_custom.yaml --relink
  python scripts\measure_tracking.py --source data\epfl_6p.avi --model models\yolo11n-pose.pt --conf 0.1 --tracker config\botsort_custom.yaml --relink --show --delay 80
  python scripts\measure_tracking.py --source data\epfl_6p.avi --model models\yolo11n-pose.pt --conf 0.1 --tracker config\botsort_custom.yaml --relink --check-swap
"""
import argparse
import os
import sys
import time
from collections import Counter

import cv2
from ultralytics import YOLO

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from src.vision.id_relink import IdRelinker  # noqa: E402
from src.vision.swap_check import SwapChecker  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True, help="영상 경로 또는 웹캠 번호(0)")
    p.add_argument("--model", default="yolov8n.pt", help="detector.yaml에 적힌 모델 경로")
    p.add_argument("--conf", type=float, default=0.5, help="track()에 넘길 conf")
    p.add_argument("--tracker", default="bytetrack.yaml", help="트래커 설정 yaml")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO 입력 크기")
    p.add_argument("--relink", action="store_true", help="IdRelinker로 ID 재연결")
    p.add_argument("--relink-gap", type=int, default=90, help="사라진 지 몇 프레임까지 재연결할지")
    p.add_argument("--relink-dist", type=float, default=0.5, help="기본 허용 거리 (박스 높이 배수)")
    p.add_argument("--relink-speed", type=float, default=0.03, help="사라진 프레임당 허용 거리 증가")
    p.add_argument("--relink-max", type=float, default=2.0, help="허용 거리 상한")
    p.add_argument("--relink-color", type=float, default=0.5, help="옷 색 차이 허용치 (0~1, 작을수록 엄격). 1이면 색 비교 끔")
    p.add_argument("--check-swap", action="store_true", help="ID 바꿔치기 의심 장면을 사진으로 저장")
    p.add_argument("--swap-dir", default="data/swap_check", help="의심 장면 사진 저장 폴더")
    p.add_argument("--swap-thresh", type=float, default=0.6, help="옷 색 차이가 이보다 크면 의심 (0~1)")
    p.add_argument("--min-frames", type=int, default=15, help="이 프레임 수 이상 유지된 ID만 '진짜 ID'로 셈")
    p.add_argument("--show", action="store_true", help="화면 표시 (q 종료, 스페이스 일시정지)")
    p.add_argument("--delay", type=int, default=1, help="--show 때 프레임 간 대기(ms). 80~100이면 느리게")
    p.add_argument("--show-from", type=int, default=1, help="--show 때 이 프레임부터 화면 표시 (앞부분은 화면 없이 빠르게 처리)")
    a = p.parse_args()

    src = int(a.source) if a.source.isdigit() else a.source
    cap = cv2.VideoCapture(src)
    if isinstance(src, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    model = YOLO(a.model)
    relinker = IdRelinker(max_gap=a.relink_gap, base_dist=a.relink_dist,
                          dist_per_unit=a.relink_speed, max_dist=a.relink_max,
                          size_ratio=(0.5, 2.0), color_thresh=a.relink_color) if a.relink else None
    checker = SwapChecker(a.swap_dir, thresh=a.swap_thresh) if a.check_swap else None
    raw_life = Counter()  # 트래커 ID별 등장 프레임 수
    life = Counter()      # 최종(표시되는) ID별 등장 프레임 수
    first_seen = {}       # 최종 ID별 처음 나온 프레임
    paused = False
    frames = 0
    t_sum = 0.0
    people_sum = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t0 = time.perf_counter()
            r = model.track(frame, persist=True, classes=[0], conf=a.conf,
                            tracker=a.tracker, imgsz=a.imgsz, verbose=False)[0]
            frames += 1

            raw_ids = [] if r.boxes.id is None else r.boxes.id.int().tolist()
            boxes = [] if r.boxes.id is None else r.boxes.xyxy.int().tolist()
            ids = relinker.update(frames, raw_ids, boxes, frame) if relinker else raw_ids
            t_sum += time.perf_counter() - t0
            if checker:
                checker.update(frames, frame, ids, boxes)

            people_sum += len(ids)
            for i in raw_ids:
                raw_life[i] += 1
            for i in ids:
                life[i] += 1
                if i not in first_seen:
                    first_seen[i] = frames
                    if a.show and frames > 1 and frames >= a.show_from:
                        print(f"[frame {frames}] 새 ID {i} 등장 (지금까지 {len(first_seen)}개)")

            if a.show and frames >= a.show_from:
                img = frame.copy()
                for (x1, y1, x2, y2), tid in zip(boxes, ids):
                    is_new = frames - first_seen[tid] < 25          # 생긴 지 25프레임 이내면 NEW
                    color = (0, 0, 255) if is_new else (
                        (tid * 67) % 256, (tid * 131) % 256, (tid * 199) % 256)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3 if is_new else 2)
                    label = f"NEW {tid}" if is_new else f"{tid}"
                    cv2.putText(img, label, (x1, max(y1 - 8, 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                cv2.putText(img, f"frame {frames}  total ids {len(first_seen)}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow("tracking", img)

                key = cv2.waitKey(0 if paused else a.delay) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    paused = not paused
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if frames == 0:
        print("프레임을 못 읽었어요. --source 경로를 확인하세요.")
        return

    long_ids = [i for i, n in life.items() if n >= a.min_frames]
    print("===== 추적 측정 결과 =====")
    print(f"source        : {a.source}")
    print(f"conf / tracker: {a.conf} / {a.tracker}  (imgsz {a.imgsz})")
    print(f"relink        : " + (f"ON (gap {a.relink_gap}, dist {a.relink_dist}, speed {a.relink_speed}, "
                                 f"max {a.relink_max}, color {a.relink_color})" if relinker else "OFF"))
    print(f"프레임 수      : {frames}")
    print(f"평균 FPS       : {frames / t_sum:.1f}  (검출+추적+재연결)")
    print(f"프레임당 평균 인원: {people_sum / frames:.2f}")
    print(f"트래커 원래 ID 개수: {len(raw_life)}")
    print(f"전체 ID 개수   : {len(life)}")
    print(f"{a.min_frames}프레임 이상 유지된 ID: {len(long_ids)}")
    print(f"가장 큰 ID 번호: {max(life) if life else '-'}")
    if relinker:
        print(f"재연결 횟수     : {relinker.relinked}")
        print(f"색이 달라서 거절: {relinker.color_rejected}")
    if checker:
        print(f"바꿔치기 의심   : {len(checker.events)}건 (사진: {a.swap_dir})")
        for f, tid, d, _ in checker.events:
            print(f"   frame {f}  ID {tid}  색 차이 {d}")
    print("ID별 유지 프레임:", dict(sorted(life.items())))


if __name__ == "__main__":
    main()