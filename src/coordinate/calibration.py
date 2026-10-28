"""좌표 변환 전체 묶음

설정 파일과 저장된 호모그래피를 불러와서 작업자 기준점(픽셀) 하나를 넣으면 실제 x(mm)와 레일 목표 좌표(mm)를 돌려줌
"""
import yaml

from src.coordinate.homography import image_to_real, load_homography
from src.coordinate.scaler import RailScaler


def load_coordinate_config(path="config/coordinate.yaml"):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


class CoordinateTransformer:
    def __init__(self, H, real_range, rail_range):
        self.H = H
        self.scaler = RailScaler(real_range, rail_range)

    @classmethod
    def from_config(cls, path="config/coordinate.yaml"):
        cfg = load_coordinate_config(path)
        H = load_homography(cfg["homography_file"])
        return cls(H, cfg["real_range"], cfg["rail_range"])

    def transform(self, ref_point):
        """기준점 (u, v) 픽셀 → (실제 x mm, 레일 목표 mm)."""
        x_real, _z = image_to_real(self.H, ref_point)   # 통로 방향 x만 사용
        x_rail = self.scaler.to_rail(x_real)
        return x_real, x_rail