from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class Detection:
    bbox: Tuple[int, int, int, int]          # (x1, y1, x2, y2)
    conf: float
    ref_point: Tuple[int, int]               # 상체 기준점 (박스 위에서 30%)
    track_id: Optional[int] = None
    keypoints: Optional[List[Tuple[float, float, float]]] = None   # 포즈 모델일 때 17개 (x, y, conf)