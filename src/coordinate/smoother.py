# 작업자별 좌표 이동 평균 (떨림 감소)

from __future__ import annotations

from collections import deque
from typing import Dict, Iterable, Tuple


class TrackSmoother:
    def __init__(self, window: int = 5, max_missing: int = 30):
        self.window = window
        self.max_missing = max_missing
        self._hist: Dict[int, deque] = {}
        self._missing: Dict[int, int] = {}

    def update(self, track_id: int, value: Tuple[float, float]) -> Tuple[float, float]:
        # (x_real, x_rail)을 넣으면 이동 평균된 (x_real, x_rail)을 돌려줌
        hist = self._hist.setdefault(track_id, deque(maxlen=self.window))
        hist.append(value)
        self._missing[track_id] = 0
        n = len(hist)
        return (sum(v[0] for v in hist) / n, sum(v[1] for v in hist) / n)

    def end_frame(self, seen_ids: Iterable[int]) -> None:
        # 프레임 끝에 호출: 이번 프레임에 안 보인 ID는 missing 카운트 증가.
        seen = set(seen_ids)
        for tid in list(self._hist):
            if tid in seen:
                continue
            self._missing[tid] = self._missing.get(tid, 0) + 1
            if self._missing[tid] > self.max_missing:
                del self._hist[tid]
                del self._missing[tid]