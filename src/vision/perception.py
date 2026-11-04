"""인식부 -> 통합부 전달 모듈 (노트북 안에서 함수로 호출)

프레임 1장을 넣으면 작업자별 {track_id, rail_mm, bbox}를 돌려줌
- track_id : ByteTrack ID (통합부 Worker.wid)
- rail_mm  : 이동 평균된 레일 좌표(mm)
- bbox     : (x1, y1, x2, y2) 픽셀, 통합부 깊이 추정(bbox 높이)용
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import List, Tuple

from src.coordinate.calibration import CoordinateTransformer
from src.coordinate.smoother import TrackSmoother
from src.vision.tracker import PersonTracker


@dataclass
class WorkerObs:
    track_id: int
    rail_mm: float
    bbox: Tuple[int, int, int, int]

    def to_dict(self) -> dict:
        return asdict(self)


class Perception:
    def __init__(self, tracker: PersonTracker, transformer: CoordinateTransformer,
                 window: int = 5):
        self.tracker = tracker
        self.transformer = transformer
        self.smoother = TrackSmoother(window=window)

    @classmethod
    def from_config(cls, detector_cfg: str = "config/detector.yaml",
                    coordinate_cfg: str = "config/coordinate.yaml") -> "Perception":
        return cls(PersonTracker.from_config(detector_cfg),
                   CoordinateTransformer.from_config(coordinate_cfg))

    def update(self, frame) -> List[WorkerObs]:
        """프레임 1장 → 이번 프레임에 보인 작업자 목록 (ID 없는 검출은 제외)."""
        out, seen = [], []
        for d in self.tracker.track(frame):
            if d.track_id is None:
                continue
            x_real, x_rail = self.transformer.transform(d.ref_point)
            _, s_rail = self.smoother.update(d.track_id, (x_real, x_rail))
            out.append(WorkerObs(d.track_id, round(s_rail, 1), d.bbox))
            seen.append(d.track_id)
        self.smoother.end_frame(seen)
        return out