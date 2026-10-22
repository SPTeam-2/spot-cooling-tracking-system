"""카메라 입력 테스트.

실행 (레포 최상위 폴더에서):
    python scripts/test_camera.py
    python scripts/test_camera.py --index 1   # USB 웹캠 등 다른 카메라

화면에 실시간 FPS가 표시되고, q 를 누르면 종료된다.
s 를 누르면 현재 화면을 data/ 폴더에 저장한다 (보고서용 캡처).
"""
import argparse
import os
import sys
import time

import cv2

# 레포 최상위 폴더를 import 경로에 추가 (src 패키지를 찾기 위함)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.vision.camera import Camera  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=int, default=None, help="카메라 번호 (기본: config 값)")
    args = parser.parse_args()

    cam = Camera.from_config("config/camera.yaml")
    if args.index is not None:
        cam.index = args.index

    with cam:
        w, h = cam.actual_size()
        print(f"카메라 {cam.index}번 열림 / 실제 해상도: {w}x{h}")

        frame_count = 0
        fail_count = 0
        start = time.time()
        fps = 0.0

        while True:
            frame = cam.read()
            if frame is None:
                fail_count += 1
                print("프레임 읽기 실패", fail_count)
                if fail_count >= 10:
                    print("연속 실패가 많아 종료합니다.")
                    break
                continue
            fail_count = 0

            # 1초마다 FPS 계산
            frame_count += 1
            elapsed = time.time() - start
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                start = time.time()

            cv2.putText(frame, f"FPS: {fps:.1f}  {w}x{h}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("test_camera (q: quit, s: save)", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                os.makedirs("data", exist_ok=True)
                path = f"data/capture_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(path, frame)
                print("저장:", path)

    cv2.destroyAllWindows()
    print(f"마지막 FPS: {fps:.1f}")


if __name__ == "__main__":
    main()