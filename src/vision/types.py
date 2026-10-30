"""인식부에서 쓰는 데이터 형식."""
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class Detection:
    """사람 1명의 검출 결과."""
    bbox: Tuple[int, int, int, int]   # 바운딩 박스 (x1, y1, x2, y2), 픽셀
    conf: float                       # 확신도 0~1
    ref_point: Tuple[int, int]        # 작업자 기준점 (x, y), 픽셀 — 좌표 변환에 사용
    track_id: Optional[int] = None    # 작업자 ID (추적 단계에서 채움, 지금은 None)