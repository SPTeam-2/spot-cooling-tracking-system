"""인식부 → 통합부 전달 모듈 (노트북 안에서 함수로 호출)

프레임 1장을 넣으면 작업자별 정보를 돌려준다.
- track_id    : 안정 ID (통합부 Worker.wid)
                트래커(BoT-SORT) ID를 IdRelinker로 재연결한 값.
                가려졌다 다시 나타나도 같은 사람이면 같은 번호 유지 (#18)
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
import os
import time
from dataclasses import asdict, dataclass
from typing import List, Optional, Tuple

import yaml

from src.coordinate.calibration import CoordinateTransformer
from src.coordinate.smoother import TrackSmoother
from src.vision.id_relink import IdRelinker
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
                 window: int = 5, relinker: Optional[IdRelinker] = None):
        self.tracker = tracker
        self.transformer = transformer
        self.relinker = relinker                                # None이면 트래커 ID 그대로 사용
        self.pos_smoother = TrackSmoother(window=window)        # (x_real, x_rail)
        self.size_smoother = TrackSmoother(window=window)       # (bbox 폭, bbox 높이)
        self.shoulder_smoother = TrackSmoother(window=window)   # (어깨 너비, 0)

    @classmethod
    def from_config(cls, detector_cfg: str = "config/detector.yaml",
                    coordinate_cfg: str = "config/coordinate.yaml",
                    relink_cfg: str = "config/relink.yaml") -> "Perception":
        relinker = None
        if os.path.exists(relink_cfg):
            with open(relink_cfg, encoding="utf-8") as f:
                rc = yaml.safe_load(f) or {}
            if rc.get("enabled", True):
                relinker = IdRelinker(
                    max_gap=rc.get("max_gap_s", 6.0),
                    base_dist=rc.get("base_dist", 0.8),
                    dist_per_unit=rc.get("dist_per_s", 1.25),
                    max_dist=rc.get("max_dist", 3.0),
                    size_ratio=tuple(rc.get("size_ratio", [0.5, 2.0])),
                    min_life=rc.get("min_life", 8),
                    color_thresh=rc.get("color_thresh", 0.5),
                )
        return cls(PersonTracker.from_config(detector_cfg),
                   CoordinateTransformer.from_config(coordinate_cfg),
                   relinker=relinker)

    def update(self, frame) -> List[WorkerObs]:
        """프레임 1장 → 이번 프레임에 보인 작업자 목록 (ID 없는 검출은 제외)."""
        out, seen, seen_sh = [], [], []
        dets = [d for d in self.tracker.track(frame) if d.track_id is not None]

        # 트래커 ID → 안정 ID (가려졌다 다시 잡혀도 같은 사람이면 같은 번호)
        raw_ids = [d.track_id for d in dets]
        if self.relinker is not None:
            boxes = [tuple(int(v) for v in d.bbox) for d in dets]
            ids = self.relinker.update(time.monotonic(), raw_ids, boxes, frame)
        else:
            ids = raw_ids

        for d, tid in zip(dets, ids):
            x1, y1, x2, y2 = d.bbox
            sh = _shoulders(d)

            # 레일 기준점: 박스 상체 기준점 (어깨 가운데는 몸 기울임에 더 흔들려서 사용 안 함)
            x_real, x_rail = self.transformer.transform(d.ref_point)
            _, s_rail = self.pos_smoother.update(tid, (x_real, x_rail))
            s_w, s_h = self.size_smoother.update(tid, (x2 - x1, y2 - y1))

            s_sh = None
            if sh is not None:
                (lx, ly), (rx, ry) = sh
                s_sh, _ = self.shoulder_smoother.update(tid, (math.hypot(rx - lx, ry - ly), 0.0))
                seen_sh.append(tid)
            elif tid in self.shoulder_smoother._hist:      # 이번만 안 보이면 이전 평균 유지
                hist = self.shoulder_smoother._hist[tid]
                s_sh = sum(v[0] for v in hist) / len(hist)

            out.append(WorkerObs(
                tid, round(s_rail, 1), d.bbox, round(s_w, 1), round(s_h, 1),
                None if s_sh is None else round(s_sh, 1),
                None if sh is None else (tuple(map(int, sh[0])), tuple(map(int, sh[1]))),
            ))
            seen.append(tid)
        self.pos_smoother.end_frame(seen)
        self.size_smoother.end_frame(seen)
        self.shoulder_smoother.end_frame(seen)
        return out