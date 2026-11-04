"""다중 작업자 송풍 우선순위 알고리즘.

단계식(lexicographic) 선정 — 앞 기준이 임계값 이내로 비슷하면 다음 기준으로:
    1) 누적 송풍시간 적은 사람  2) 오래 기다린 사람  3) 캐리지에서 가까운 사람
임계값(cooled_similar_s / wait_similar_s)이 공정성↔효율성 다이얼. → config/coordinate.yaml
distance_mm은 캐리지↔작업자 레일거리(이동 최소화용)이며, 팬 세기용 깊이와 무관하다.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Worker:
    wid: int
    distance_mm: float = 0.0     # 캐리지↔작업자 레일상 거리(mm)
    cooled_total_s: float = 0.0  # 누적 송풍받은 시간
    wait_s: float = 0.0          # 마지막 송풍 이후 대기 시간
    detected: bool = True        # 현재 프레임에 검출됐는지
    cooldown_until: float = 0.0  # 이 시각까지 재선정 제외


def pick_target(
    workers: list[Worker],
    now: float,
    *,
    cooled_similar_s: float = 20.0,
    wait_similar_s: float = 20.0,
) -> Worker | None:
    """송풍 대상 1명 선정. 후보 없으면 None. now는 Cooldown 판정용 현재 시각(초)."""
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
