import yaml
from ultralytics import YOLO

from src.vision.types import Detection

PERSON_CLASS = 0  # COCO 데이터셋에서 0번 클래스 = person


class PersonDetector:
    def __init__(self, model="models/yolo11n.pt", conf=0.5, imgsz=640, ref_y_ratio=0.3):
        self.model = YOLO(model)
        self.conf = conf
        self.imgsz = imgsz
        self.ref_y_ratio = ref_y_ratio

    @classmethod
    def from_config(cls, path="config/detector.yaml"):
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        return cls(**cfg)

    def detect(self, frame):
        """프레임에서 사람만 검출해 Detection 리스트로 반환 (없으면 빈 리스트)."""
        results = self.model.predict(
            frame,
            classes=[PERSON_CLASS],  # 사람만 검출
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False,
        )
        detections = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            detections.append(Detection(
                bbox=(x1, y1, x2, y2),
                conf=float(box.conf[0]),
                ref_point=self._ref_point(x1, y1, x2, y2),
            ))
        return detections

    def _ref_point(self, x1, y1, x2, y2):
        """작업자 기준점: 가로는 박스 중앙, 세로는 상체 높이.
        책상 등에 다리가 가려질 수 있어 발(박스 아래) 대신 상체를 기준으로 한다."""
        x = (x1 + x2) // 2
        y = int(y1 + (y2 - y1) * self.ref_y_ratio)
        return x, y