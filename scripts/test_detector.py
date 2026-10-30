import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.vision.camera import Camera  # noqa: E402
from src.vision.detector import PersonDetector  # noqa: E402


def draw(frame, detections, fps):
    for d in detections:
        x1, y1, x2, y2 = d.bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, d.ref_point, 6, (0, 0, 255), -1)
        cv2.putText(frame, f"{d.conf:.2f} ({d.ref_point[0]},{d.ref_point[1]})",
                    (x1, max(y1 - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"FPS: {fps:.1f}  people: {len(detections)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=int, default=None, help="카메라 번호 (기본: config 값)")
    args = parser.parse_args()

    cam = Camera.from_config("config/camera.yaml")
    if args.index is not None:
        cam.index = args.index
    detector = PersonDetector.from_config("config/detector.yaml")

    total_frames = 0
    detected_frames = 0
    frame_count = 0
    fps = 0.0
    start = time.time()

    with cam:
        while True:
            frame = cam.read()
            if frame is None:
                print("프레임 읽기 실패")
                break

            detections = detector.detect(frame)

            # 검출률 계산용 (사람이 화면에 있을 때 테스트하면 검출 성공률이 됨)
            total_frames += 1
            if detections:
                detected_frames += 1

            frame_count += 1
            elapsed = time.time() - start
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                start = time.time()

            draw(frame, detections, fps)
            cv2.imshow("test_detector (q: quit, s: save)", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                os.makedirs("data", exist_ok=True)
                path = f"data/detect_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(path, frame)
                print("저장:", path)

    cv2.destroyAllWindows()
    if total_frames:
        rate = detected_frames / total_frames * 100
        print(f"마지막 FPS: {fps:.1f}")
        print(f"검출된 프레임: {detected_frames}/{total_frames} ({rate:.1f}%)")


if __name__ == "__main__":
    main()