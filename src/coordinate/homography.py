"""호모그래피 변환

화면 좌표(픽셀)를 실제 좌표(mm)로 바꾸는 행렬 H를 계산 후 저장/불러오기 실행
기준점 4개 이상의 (화면 좌표, 실제 좌표) 쌍 필요
"""
import os

import cv2
import numpy as np


def compute_homography(image_points, real_points):
    """화면 좌표 리스트와 실제 좌표 리스트로 H를 계산한다."""
    if len(image_points) < 4 or len(image_points) != len(real_points):
        raise ValueError("기준점은 4개 이상, 화면 좌표와 실제 좌표 개수가 같아야 합니다.")
    src = np.array(image_points, dtype=np.float32)
    dst = np.array(real_points, dtype=np.float32)
    H, _ = cv2.findHomography(src, dst)
    if H is None:
        raise RuntimeError("호모그래피 계산 실패: 기준점이 한 줄로 놓여 있지 않은지 확인하세요.")
    return H


def image_to_real(H, point):
    """화면 좌표 (u, v) 하나를 실제 좌표 (x, z) mm로 바꾼다."""
    p = np.array([[point]], dtype=np.float32)          # 모양 (1, 1, 2)
    x, z = cv2.perspectiveTransform(p, H)[0][0]
    return float(x), float(z)


def save_homography(path, H, image_points, real_points):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez(path, H=H, image_points=np.array(image_points), real_points=np.array(real_points))


def load_homography(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} 없음: 먼저 scripts/calibrate_camera.py를 실행하세요.")
    return np.load(path)["H"]