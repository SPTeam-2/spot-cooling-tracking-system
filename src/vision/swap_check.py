"""
ID 바꿔치기 의심 장면 자동 검출
- ID마다 '혼자 있을 때'의 옷 색을 기억해 두고,
  같은 ID인데 옷 색이 갑자기 크게 달라지면(여러 프레임 연속) 바꿔치기 의심으로 기록
- 의심 장면마다 [이전 모습 | 지금 모습] 사진을 저장 → 사진만 보고 같은 사람인지 판단
"""
import os

import cv2
import numpy as np

from src.vision.id_relink import color_hist


def _crop(frame, box, height=220):
    h_img, w_img = frame.shape[:2]
    x1, y1, x2, y2 = box
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w_img, x2), min(h_img, y2)
    if x2 - x1 < 4 or y2 - y1 < 4:
        return None
    c = frame[y1:y2, x1:x2]
    s = height / c.shape[0]
    return cv2.resize(c, (max(1, int(c.shape[1] * s)), height))


def _isolated(k, boxes):
    x1, y1, x2, y2 = boxes[k]
    for j, (a1, b1, a2, b2) in enumerate(boxes):
        if j != k and min(x2, a2) > max(x1, a1) and min(y2, b2) > max(y1, b1):
            return False
    return True


class SwapChecker:
    def __init__(self, out_dir, thresh=0.6, confirm=5):
        self.out_dir = out_dir
        self.thresh = thresh        # 옷 색 차이(0=같음, 1=다름)가 이보다 크면 '달라짐'
        self.confirm = confirm      # 이 프레임 연속으로 달라야 의심으로 기록 (순간 노이즈 무시)
        self.ref = {}               # ID -> 기억한 색
        self.ref_crop = {}          # ID -> 기억한 모습(사진)
        self.ref_frame = {}         # ID -> 그 모습을 찍은 프레임
        self.bad = {}               # ID -> 연속으로 달랐던 프레임 수
        self.events = []
        os.makedirs(out_dir, exist_ok=True)

    def update(self, frame_idx, frame, ids, boxes):
        for k, tid in enumerate(ids):
            if not _isolated(k, boxes):
                continue
            hist = color_hist(frame, boxes[k])
            crop = _crop(frame, boxes[k])
            if hist is None or crop is None:
                continue
            if tid not in self.ref:
                self.ref[tid], self.ref_crop[tid], self.ref_frame[tid] = hist, crop, frame_idx
                self.bad[tid] = 0
                continue
            d = cv2.compareHist(hist, self.ref[tid], cv2.HISTCMP_BHATTACHARYYA)
            if d > self.thresh:
                self.bad[tid] += 1
                if self.bad[tid] == self.confirm:
                    self._save(frame_idx, tid, d, crop)
                    # 새 모습으로 기준을 바꿔서 같은 장면을 반복 기록하지 않음
                    self.ref[tid], self.ref_crop[tid], self.ref_frame[tid] = hist, crop, frame_idx
                    self.bad[tid] = 0
            else:
                self.bad[tid] = 0
                self.ref[tid] = (0.95 * self.ref[tid] + 0.05 * hist).astype(np.float32)
                self.ref_crop[tid], self.ref_frame[tid] = crop, frame_idx

    def _save(self, frame_idx, tid, d, crop):
        before = self.ref_crop[tid]
        gap = np.full((before.shape[0], 20, 3), 255, np.uint8)
        img = np.hstack([before, gap, crop])
        bar = np.full((40, img.shape[1], 3), 255, np.uint8)
        cv2.putText(bar, f"ID {tid}  f{self.ref_frame[tid]} -> f{frame_idx}  diff {d:.2f}", (5, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        img = np.vstack([bar, img])
        name = os.path.join(self.out_dir, f"f{frame_idx:05d}_id{tid}.jpg")
        cv2.imwrite(name, img)
        self.events.append((frame_idx, tid, round(float(d), 2), name))
        print(f"[frame {frame_idx}] ID {tid} 바꿔치기 의심 (색 차이 {d:.2f}) -> {name}")