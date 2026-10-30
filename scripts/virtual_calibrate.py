"""가상 캘리브레이션 (기준점 없이 테스트 시).

화면의 네 모서리를 실제 기준점으로 가정하고 H를 생성 → 화면 왼쪽 끝 = 0mm, 오른쪽 끝 = real_range 최대값으로 단순 비례 변환됨
코드 흐름·가상 캐리지 확인용 (원근 보정 X) 
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.coordinate.calibration import load_coordinate_config  # noqa: E402
from src.coordinate.homography import compute_homography, save_homography  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=1280, help="카메라 실제 가로 해상도")
    parser.add_argument("--height", type=int, default=720, help="카메라 실제 세로 해상도")
    args = parser.parse_args()

    cfg = load_coordinate_config("config/coordinate.yaml")
    w, h = args.width, args.height

    # real_points 순서(1 시작·바닥, 2 끝·바닥, 3 끝·위, 4 시작·위)에 맞춘 화면 네 모서리
    image_points = [(0, h), (w, h), (w, 0), (0, 0)]
    H = compute_homography(image_points, cfg["real_points"])
    save_homography(cfg["homography_file"], H, image_points, cfg["real_points"])
    print(f"가상 캘리브레이션 저장: {cfg['homography_file']} (화면 {w}x{h} 전체 = 실제 {cfg['real_range']} mm)")


if __name__ == "__main__":
    main()