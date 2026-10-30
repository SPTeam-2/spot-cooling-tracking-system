# 좌표 변환 테스트 (YOLO 검출 + 호모그래피 + 레일 좌표).

import os
import sys
import time

import cv2

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.coordinate.calibration import CoordinateTransformer, load_coordinate_config  # noqa: E402
from src.vision.camera import Camera  # noqa: E402
from src.vision.detector import PersonDetector  # noqa: E402


def draw_rail(frame, x_rail_list, rail_range):
    """화면 아래에 레일 막대와 가상 캐리지(목표 위치)를 그린다."""
    h, w = frame.shape[:2]
    left, right, y = 40, w - 40, h - 40
    cv2.line(frame, (left, y), (right, y), (200, 200, 200), 4)
    rail_min, rail_max = rail_range
    for x_rail in x_rail_list:
        px = int(left + (x_rail - rail_min) / (rail_max - rail_min) * (right - left))
        cv2.circle(frame, (px, y), 12, (255, 0, 0), -1)
        cv2.putText(frame, f"{x_rail:.0f}mm", (px - 30, y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)


def main():
    cfg = load_coordinate_config("config/coordinate.yaml")
    transformer = CoordinateTransformer.from_config("config/coordinate.yaml")
    detector = PersonDetector.from_config("config/detector.yaml")

    window = "test_coordinate (q: quit, s: save)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)   # 창 크기 조절 가능
    cv2.resizeWindow(window, 1280, 720)

    with Camera.from_config("config/camera.yaml") as cam:
        while True:
            frame = cam.read()
            if frame is None:
                break

            x_rail_list = []
            for d in detector.detect(frame):
                x_real, x_rail = transformer.transform(d.ref_point)
                x_rail_list.append(x_rail)

                x1, y1, x2, y2 = d.bbox
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, d.ref_point, 6, (0, 0, 255), -1)
                cv2.putText(frame, f"real {x_real:.0f}mm -> rail {x_rail:.0f}mm",
                            (x1, max(y1 - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            draw_rail(frame, x_rail_list, cfg["rail_range"])
            cv2.imshow(window, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                os.makedirs("data", exist_ok=True)
                path = f"data/coord_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(path, frame)
                print("저장:", path)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()