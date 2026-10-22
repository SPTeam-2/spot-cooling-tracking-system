import platform
import time

import cv2
import yaml


class Camera:
    def __init__(self, index=0, width=1280, height=720, fps=30):
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None

    @classmethod
    def from_config(cls, path="config/camera.yaml"):
        """설정 파일(yaml)에서 값을 읽어 Camera를 만든다."""
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        return cls(
            index=cfg.get("index", 0),
            width=cfg.get("width", 1280),
            height=cfg.get("height", 720),
            fps=cfg.get("fps", 30),
        )

    def open(self):
        """카메라를 연다. 실패하면 RuntimeError."""
        if platform.system() == "Windows":
            self.cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(self.index)

        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {self.index}번을 열 수 없습니다. index 값을 확인하세요.")

        # 원하는 해상도/FPS 요청 (카메라가 지원하지 않으면 가까운 값으로 설정됨)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return self

    def read(self, retries=3):
        """프레임 1장을 읽는다. 실패하면 몇 번 다시 시도하고, 끝내 실패하면 None."""
        if self.cap is None:
            raise RuntimeError("open()을 먼저 호출하세요.")
        for _ in range(retries):
            ok, frame = self.cap.read()
            if ok and frame is not None:
                return frame
            time.sleep(0.05)
        return None

    def actual_size(self):
        """카메라가 실제로 적용한 해상도(가로, 세로)."""
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    # 자동으로 카메라를 닫음
    def __enter__(self):
        return self.open()

    def __exit__(self, *args):
        self.release()