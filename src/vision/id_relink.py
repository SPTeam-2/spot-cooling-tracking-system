"""
트래커 ID → 안정 ID 재연결
- 새 ID가 생기면, 최근 사라진 ID 중 가깝고 옷 색이 비슷한 것을 이어서 씀
- 처음 보는 사람은 confirm_n 프레임 연속으로 잡혀야 번호를 줌 (깜빡인 박스 제외)
- 이미 보이는 사람 바로 위에 박스가 생기면(겹침/중복) dup_wait 동안 번호를 안 줌(새 번호·재연결 모두).
  그 사이 원래 박스가 사라지면 원래 번호를 이어받고, 떨어지면 그때 색까지 보고 결정
- 시간 단위는 update()의 t를 따름 (측정 스크립트: 프레임, perception: 초)
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
    def __init__(self, max_gap=90, base_dist=0.5, dist_per_unit=0.03,
                 max_dist=2.0, size_ratio=(0.6, 1.6), min_life=15, color_thresh=0.5,
                 confirm_n=3, dup_dist=0.25, dup_wait=0, debug=False):
        self.max_gap = max_gap               # 사라진 지 이 시간(t 단위) 이내만 재연결 후보
        self.base_dist = base_dist           # 기본 허용 거리 (박스 높이 배수)
        self.dist_per_unit = dist_per_unit   # 사라진 시간(t 단위)만큼 허용 거리 증가
        self.max_dist = max_dist             # 허용 거리 상한
        self.size_ratio = size_ratio         # 박스 높이 비율 허용 범위
        self.min_life = min_life             # 이 횟수 이상 관측된 ID만 이어받기 후보
        self.color_thresh = color_thresh     # 색 차이(0=같음, 1=다름)가 이보다 작아야 재연결
        self.confirm_n = confirm_n           # 새 사람은 이 프레임 연속으로 잡혀야 번호 부여
        self.pending = {}                    # 아직 번호 없는 트래커 ID -> 연속으로 잡힌 횟수
        self.dup_dist = dup_dist             # 보이는 사람과 이보다 가까우면(박스 높이 배수) 겹친 박스로 봄
        self.dup_wait = dup_wait             # 겹친 박스에 번호를 보류하는 최대 시간 (t 단위)
        self.dup_start = {}                  # 보류 중인 트래커 ID -> 보류 시작 t
        self.dup_held = 0                    # 겹친 박스를 보류한 횟수
        self.raw2stable = {}                 # 트래커 ID -> 안정 ID
        self.last = {}                       # 안정 ID -> (t, cx, cy, h)
        self.life = {}                       # 안정 ID -> 등장 프레임 수
        self.hist = {}                       # 안정 ID -> 색 히스토그램 (천천히 갱신)
        self.next_id = 1
        self.relinked = 0                    # 재연결한 횟수
        self.color_rejected = 0              # 위치는 맞는데 색이 달라서 거절한 횟수
        self.debug = debug                   # True면 새 번호·재연결·보류를 이유와 함께 출력
        self.tag = ""                        # 디버그 출력 앞에 붙일 표시 (예: 프레임 번호)

    def update(self, t, raw_ids, boxes_xyxy, frame=None):
        """t(프레임 번호 또는 초)와 raw_ids, boxes_xyxy를 받아 같은 순서로 안정 ID 리스트를 돌려줌.
        아직 확정 안 된 사람은 None. frame을 주면 색 비교도 함"""
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
            dup_of = None                              # 바로 겹친 '지금 보이는' 사람
            why = []                                   # 디버그: 주변 후보와 거절 이유
            for sid, (f, px, py, ph) in self.last.items():
                d = math.hypot(cx - px, cy - py) / ph
                gap = t - f
                if sid in used:
                    why.append(f"{sid}:지금도 보임(d={d:.1f})")
                    if d < self.dup_dist and 0.67 <= h / ph <= 1.5:
                        dup_of = sid
                    continue
                if self.life.get(sid, 0) < self.min_life:
                    why.append(f"{sid}:관측 적음({self.life.get(sid, 0)})")
                    continue
                if gap <= 0 or gap > self.max_gap:
                    why.append(f"{sid}:시간 초과(gap={gap:.1f})")
                    continue
                r = h / ph
                if not (self.size_ratio[0] <= r <= self.size_ratio[1]):
                    why.append(f"{sid}:크기 다름(r={r:.2f}, d={d:.1f})")
                    continue
                limit = min(self.base_dist + self.dist_per_unit * gap, self.max_dist)
                if d > limit:
                    why.append(f"{sid}:멀음(d={d:.1f}>{limit:.1f})")
                    continue
                if hists[k] is not None and self.hist.get(sid) is not None:
                    c = cv2.compareHist(hists[k], self.hist[sid], cv2.HISTCMP_BHATTACHARYYA)
                    if c > self.color_thresh:
                        self.color_rejected += 1
                        why.append(f"{sid}:색 다름(c={c:.2f}, d={d:.1f})")
                        continue
                if d < best_d:
                    best, best_d = sid, d
            rid = raw_ids[k]
            self.pending[rid] = self.pending.get(rid, 0) + 1

            # 보이는 사람 바로 위에 겹친 박스 → 새 번호든 재연결이든 dup_wait 동안 보류
            if dup_of is not None and self.dup_wait > 0:
                if rid not in self.dup_start:
                    self.dup_start[rid] = t
                    self.dup_held += 1
                    if self.debug:
                        print(f"{self.tag} [보류] 트래커 {rid}: 보이는 {dup_of}번 위에 겹침")
                if t - self.dup_start[rid] < self.dup_wait:
                    continue

            if best is not None:                       # 사라졌던 사람 → 원래 번호
                self.relinked += 1
                if self.debug:
                    f0, px, py, ph = self.last[best]
                    c = (cv2.compareHist(hists[k], self.hist[best], cv2.HISTCMP_BHATTACHARYYA)
                         if hists[k] is not None and self.hist.get(best) is not None else -1)
                    print(f"{self.tag} [재연결] 트래커 {rid} → 기존 번호 {best} "
                          f"(거리 {best_d:.2f}, 사라진 시간 {t - f0:.1f}, 색 차이 {c:.2f}) / 다른 후보: {', '.join(why) or '없음'}")
            else:                                      # 처음 보는 사람 → 연속 confirm_n 프레임 확인
                if self.pending[rid] < self.confirm_n:
                    continue                           # 아직 확정 전: out[k] = None
                best = self.next_id
                self.next_id += 1
                if self.debug:
                    print(f"{self.tag} [새 번호] {best} (트래커 {rid}) / 주변: {', '.join(why) or '없음'}")
            self.pending.pop(rid, None)
            self.dup_start.pop(rid, None)
            self.raw2stable[rid] = best
            out[k] = best
            used.add(best)

        # 이번 프레임에 안 보인 확정 전 ID는 카운트 초기화 (연속으로 잡혀야 함)
        seen_raw = set(raw_ids)
        self.pending = {r: c for r, c in self.pending.items() if r in seen_raw}
        self.dup_start = {r: v for r, v in self.dup_start.items() if r in seen_raw}

        # 3) 위치·수명·색 갱신 + 오래된 기록 정리
        for k, sid in enumerate(out):
            if sid is None:
                continue
            self.last[sid] = (t, *geo[k])
            self.life[sid] = self.life.get(sid, 0) + 1
            if hists[k] is not None and self._isolated(k, boxes_xyxy):
                old = self.hist.get(sid)
                self.hist[sid] = hists[k] if old is None else (0.9 * old + 0.1 * hists[k]).astype(np.float32)
        for sid in [s for s, v in self.last.items() if t - v[0] > self.max_gap]:
            del self.last[sid]
        return out

    @staticmethod
    def _isolated(k, boxes):
        """다른 박스와 실제로 겹치지 않을 때만 색을 갱신 (겹치면 남의 옷 색이 섞임)"""
        x1, y1, x2, y2 = boxes[k]
        for j, (a1, b1, a2, b2) in enumerate(boxes):
            if j != k and a1 < x2 and a2 > x1 and b1 < y2 and b2 > y1:
                return False
        return True