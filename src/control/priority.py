from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Worker:
    wid: int
    distance_mm: float = 0.0     # 캐리지↔작업자 레일거리 (팬 세기용 깊이와 무관)
    cooled_total_s: float = 0.0
    wait_s: float = 0.0
    detected: bool = True
    cooldown_until: float = 0.0


def pick_target(
    workers: list[Worker],
    now: float,  # cooldown 판정용 현재 시각(초)
    *,
    cooled_similar_s: float = 20.0,
    wait_similar_s: float = 20.0,
) -> Worker | None:
    cands = [w for w in workers if w.detected and now >= w.cooldown_until]
    if len(cands) <= 1:
        return cands[0] if cands else None

    min_cooled = min(w.cooled_total_s for w in cands)
    tier1 = [w for w in cands if w.cooled_total_s - min_cooled <= cooled_similar_s]
    if len(tier1) == 1:
        return tier1[0]

    max_wait = max(w.wait_s for w in tier1)
    tier2 = [w for w in tier1 if max_wait - w.wait_s <= wait_similar_s]
    if len(tier2) == 1:
        return tier2[0]

    return min(tier2, key=lambda w: w.distance_mm)
