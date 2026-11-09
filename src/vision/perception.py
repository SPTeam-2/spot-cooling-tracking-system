"""인식부 → 통합부 전달 모듈 (노트북 안에서 함수로 호출)

프레임 1장을 넣으면 작업자별 정보를 돌려준다.
- track_id    : ByteTrack ID (통합부 Worker.wid)
- rail_mm     : 이동 평균된 레일 좌표(mm), 박스 상체 기준점(위에서 30%) 기준
- bbox        : (x1, y1, x2, y2) 이번 프레임 원본 박스(px)
- width_px    : 이동 평균된 박스 폭(px)
- height_px   : 이동 평균된 박스 높이(px)
- shoulder_px : 이동 평균된 어깨 너비(px) → 깊이 추정 추천값 (팔을 뻗어도 거의 안 변함)
                어깨가 안 보인 적만 있으면 None
- shoulders   : 이번 프레임 (왼쪽 어깨, 오른쪽 어깨) 좌표, 안 보이면 None (화면 표시용)
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import List, Optional, Tuple

from src.coordinate.calibration import CoordinateTransformer
from src.coordinate.smoother import TrackSmoother
from src.vision.tracker import PersonTracker

L_SHOULDER, R_SHOULDER = 5, 6     # COCO 키포인트 번호
KP_CONF_MIN = 0.5                 # 이 값보다 확신이 낮은 어깨는 안 쓴 것으로 처리


@dataclass
class WorkerObs:
    track_id: int
    rail_mm: float
    bbox: Tuple[int, int, int, int]
    width_px: float
    height_px: float
    shoulder_px: Optional[float] = None
    shoulders: Optional[Tuple[Tuple[int, int], Tuple[int, int]]] = None

    def to_dict(self) -> dict:
        return asdict(self)


def _shoulders(det) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """양쪽 어깨가 둘 다 확실히 보이면 좌표 반환, 아니면 None."""
    if not det.keypoints or len(det.keypoints) <= R_SHOULDER:
        return None
    lx, ly, lc = det.keypoints[L_SHOULDER]
    rx, ry, rc = det.keypoints[R_SHOULDER]
    if lc < KP_CONF_MIN or rc < KP_CONF_MIN:
        return None
    return (lx, ly), (rx, ry)


class Perception:
    def __init__(self, tracker: PersonTracker, transformer: CoordinateTransformer,
                 window: int = 5):
        self.tracker = tracker
        self.transformer = transformer
        self.pos_smoother = TrackSmoother(window=window)        # (x_real, x_rail)
        self.size_smoother = TrackSmoother(window=window)       # (bbox 폭, bbox 높이)
        self.shoulder_smoother = TrackSmoother(window=window)   # (어깨 너비, 0)

    @classmethod
    def from_config(cls, detector_cfg: str = "config/detector.yaml",
                    coordinate_cfg: str = "config/coordinate.yaml") -> "Perception":
        return cls(PersonTracker.from_config(detector_cfg),
                   CoordinateTransformer.from_config(coordinate_cfg))

    def update(self, frame) -> List[WorkerObs]:
        """프레임 1장 → 이번 프레임에 보인 작업자 목록 (ID 없는 검출은 제외)."""
        out, seen, seen_sh = [], [], []
        for d in self.tracker.track(frame):
            if d.track_id is None:
                continue
            x1, y1, x2, y2 = d.bbox
            sh = _shoulders(d)

            # 레일 기준점: 박스 상체 기준점 (어깨 가운데는 몸 기울임에 더 흔들려서 사용 안 함)
            x_real, x_rail = self.transformer.transform(d.ref_point)
            _, s_rail = self.pos_smoother.update(d.track_id, (x_real, x_rail))
            s_w, s_h = self.size_smoother.update(d.track_id, (x2 - x1, y2 - y1))

            s_sh = None
            if sh is not None:
                (lx, ly), (rx, ry) = sh
                s_sh, _ = self.shoulder_smoother.update(d.track_id, (math.hypot(rx - lx, ry - ly), 0.0))
                seen_sh.append(d.track_id)
            elif d.track_id in self.shoulder_smoother._hist:      # 이번만 안 보이면 이전 평균 유지
                hist = self.shoulder_smoother._hist[d.track_id]
                s_sh = sum(v[0] for v in hist) / len(hist)

            out.append(WorkerObs(
                d.track_id, round(s_rail, 1), d.bbox, round(s_w, 1), round(s_h, 1),
                None if s_sh is None else round(s_sh, 1),
                None if sh is None else (tuple(map(int, sh[0])), tuple(map(int, sh[1]))),
            ))
            seen.append(d.track_id)
        self.pos_smoother.end_frame(seen)
        self.size_smoother.end_frame(seen)
        self.shoulder_smoother.end_frame(seen_sh)
        return out