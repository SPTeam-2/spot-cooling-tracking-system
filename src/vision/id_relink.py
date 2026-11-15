"""
트래커 raw ID -> 안정 ID 재연결
- 트래커가 새 ID를 붙여도, 최근에 사라진 ID가 근처에 있었고 옷 색이 비슷하면 그 ID를 이어서 씀
- 거리 기준은 '박스 높이' 배수라서 사람이 크게/작게 보여도 같은 기준으로 동작
- 잠깐 깜빡인 가짜 ID(min_life 프레임 미만)는 이어받을 후보에서 제외
"""
import math

import cv2
import numpy as np


def color_hist(frame, box):
    """박스 위쪽 60%(상체+허벅지)의 HSV 색 히스토그램"""
    if frame is None:
        return None
    h_img, w_img = frame.shape[:2]
    x1, y1, x2, y2 = box
    x1, x2 = max(0, x1), min(w_img, x2)
    y1 = max(0, y1)
    y2 = min(h_img, y1 + int((y2 - y1) * 0.6))
    if x2 - x1 < 4 or y2 - y1 < 4:
        return None
    hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
    cv2.normalize(hist, hist)
    return hist


class IdRelinker:
    def __init__(self, max_gap_frames=90, base_dist=0.5, dist_per_frame=0.03,
                 max_dist=2.0, size_ratio=(0.6, 1.6), min_life=15, color_thresh=0.5):
        self.max_gap = max_gap_frames        # 사라진 지 이 프레임 이내만 재연결 후보
        self.base_dist = base_dist           # 기본 허용 거리 (박스 높이 배수)
        self.dist_per_frame = dist_per_frame # 사라진 시간만큼 허용 거리 증가
        self.max_dist = max_dist             # 허용 거리 상한
        self.size_ratio = size_ratio         # 박스 높이 비율 허용 범위
        self.min_life = min_life             # 이 프레임 이상 살아있던 ID만 이어받기 후보
        self.color_thresh = color_thresh     # 색 차이(0=같음, 1=다름)가 이보다 작아야 재연결
        self.raw2stable = {}                 # 트래커 ID -> 안정 ID
        self.last = {}                       # 안정 ID -> (frame, cx, cy, h)
        self.life = {}                       # 안정 ID -> 등장 프레임 수
        self.hist = {}                       # 안정 ID -> 색 히스토그램 (천천히 갱신)
        self.next_id = 1
        self.relinked = 0                    # 재연결한 횟수
        self.color_rejected = 0              # 위치는 맞는데 색이 달라서 거절한 횟수

    def update(self, frame_idx, raw_ids, boxes_xyxy, frame=None):
        """raw_ids, boxes_xyxy 순서대로 안정 ID 리스트를 돌려줌. frame을 주면 색 비교도 함"""
        out = [None] * len(raw_ids)
        used = set()
        geo = [((x1 + x2) / 2, (y1 + y2) / 2, max(y2 - y1, 1)) for (x1, y1, x2, y2) in boxes_xyxy]
        hists = [color_hist(frame, b) for b in boxes_xyxy]

        # 1) 이미 아는 트래커 ID는 그대로 매핑
        new_k = []
        for k, rid in enumerate(raw_ids):
            sid = self.raw2stable.get(rid)
            if sid is not None and sid not in used:
                out[k] = sid
                used.add(sid)
            else:
                new_k.append(k)

        # 2) 처음 보는 트래커 ID -> 최근에 사라진 안정 ID 중 가깝고 색이 비슷한 것에 연결
        for k in new_k:
            cx, cy, h = geo[k]
            best, best_d = None, float("inf")
            for sid, (f, px, py, ph) in self.last.items():
                if sid in used or self.life.get(sid, 0) < self.min_life:
                    continue
                gap = frame_idx - f
                if gap < 1 or gap > self.max_gap:
                    continue
                r = h / ph
                if not (self.size_ratio[0] <= r <= self.size_ratio[1]):
                    continue
                d = math.hypot(cx - px, cy - py) / ph
                limit = min(self.base_dist + self.dist_per_frame * gap, self.max_dist)
                if d > limit:
                    continue
                if hists[k] is not None and self.hist.get(sid) is not None:
                    c = cv2.compareHist(hists[k], self.hist[sid], cv2.HISTCMP_BHATTACHARYYA)
                    if c > self.color_thresh:
                        self.color_rejected += 1
                        continue
                if d < best_d:
                    best, best_d = sid, d
            if best is None:
                best = self.next_id
                self.next_id += 1
            else:
                self.relinked += 1
            self.raw2stable[raw_ids[k]] = best
            out[k] = best
            used.add(best)

        # 3) 위치·수명·색 갱신 + 오래된 기록 정리
        for k, sid in enumerate(out):
            self.last[sid] = (frame_idx, *geo[k])
            self.life[sid] = self.life.get(sid, 0) + 1
            if hists[k] is not None and self._isolated(k, geo):
                old = self.hist.get(sid)
                self.hist[sid] = hists[k] if old is None else (0.9 * old + 0.1 * hists[k]).astype(np.float32)
        for sid in [s for s, v in self.last.items() if frame_idx - v[0] > self.max_gap]:
            del self.last[sid]
        return out

    @staticmethod
    def _isolated(k, geo):
        """다른 사람과 겹치지 않을 때만 색을 갱신 (겹치면 남의 옷 색이 섞임)"""
        cx, cy, h = geo[k]
        for j, (ox, oy, oh) in enumerate(geo):
            if j != k and abs(cx - ox) < 0.6 * max(h, oh) and abs(cy - oy) < 0.8 * max(h, oh):
                return False
        return True