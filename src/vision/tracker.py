# 작업자 추적 모듈 (YOLO11n + ByteTrack)

from __future__ import annotations

from typing import List

import yaml
from ultralytics import YOLO

from .types import Detection


class PersonTracker:
    def __init__(
        self,
        model: str = "models/yolo11n.pt",
        conf: float = 0.5,
        imgsz: int = 640,
        ref_y_ratio: float = 0.3,
        tracker: str = "bytetrack.yaml",
    ):
        self.model = YOLO(model)
        self.conf = conf
        self.imgsz = imgsz
        self.ref_y_ratio = ref_y_ratio
        self.tracker = tracker

    @classmethod
    def from_config(cls, path: str = "config/detector.yaml") -> "PersonTracker":
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        return cls(
            model=cfg.get("model", "models/yolo11n.pt"),
            conf=cfg.get("conf", 0.5),
            imgsz=cfg.get("imgsz", 640),
            ref_y_ratio=cfg.get("ref_y_ratio", 0.3),
            tracker=cfg.get("tracker", "bytetrack.yaml"),
        )

    def track(self, frame) -> List[Detection]:

        # 한 프레임을 넣으면 ID가 붙은 Detection 목록을 돌려줌.
        # persist=True: 이전 프레임의 추적 상태를 이어서 사용 (같은 사람 = 같은 ID)
        results = self.model.track(
            frame,
            persist=True,
            tracker=self.tracker,
            classes=[0],          # 사람만
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False,
        )
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []

        # 추적이 막 시작된 프레임에는 id가 없을 수 있음 → None
        ids = boxes.id.int().tolist() if boxes.id is not None else [None] * len(boxes)

        # 포즈 모델이면 사람마다 관절 17개 (x, y, 확신도). 일반 모델이면 None
        kps = results[0].keypoints
        if kps is not None and kps.xy is not None:
            kxy = kps.xy.tolist()
            kcf = kps.conf.tolist() if kps.conf is not None else [[1.0] * len(k) for k in kxy]
            keypoints = [[(x, y, c) for (x, y), c in zip(pts, cs)] for pts, cs in zip(kxy, kcf)]
        else:
            keypoints = [None] * len(boxes)

        detections = []
        for xyxy, conf, tid, kp in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), ids, keypoints):
            x1, y1, x2, y2 = map(int, xyxy)
            ref = ((x1 + x2) // 2, int(y1 + (y2 - y1) * self.ref_y_ratio))
            detections.append(
                Detection(bbox=(x1, y1, x2, y2), conf=float(conf), ref_point=ref,
                          track_id=tid, keypoints=kp)
            )
        return detections