"""다중 작업자 송풍 우선순위 알고리즘

여러 명이 동시에 검출됐을 때 "누구에게 먼저 송풍할지"를 정한다.
단계식(lexicographic) 규칙 — 앞 기준이 비슷하면(임계값 이내) 다음 기준으로:

    1순위  누적 송풍시간  (적게 받은 사람 먼저)       → 공정성
    2순위  대기 시간      (오래 기다린 사람 먼저)      → 공정성
    3순위  캐리지-작업자 거리 (가까운 사람 먼저)       → 효율(캐리지 이동 최소)

유사 임계값(cooled_similar_s / wait_similar_s)이 공정성↔효율성 다이얼이다.
값이 클수록 "비슷하게 받았으면 가까운 사람 먼저"가 자주 발동해 캐리지 이동이
줄고(효율↑), 작을수록 송풍시간 균등에 민감해진다(공정성↑). 기본 20s는 시뮬레이션
균형점(캐리지 이동 약 18%↓, 5명까지 공정성 편차 20% 목표 이내). → config/coordinate.yaml

추가 규칙:
    - Cooldown      : 방금 송풍받은 대상은 일정 시간 후보에서 제외 → 바로 재선정 방지
    - 최소 유지시간 : 한 대상에게 최소 T초 송풍한 뒤 전환 → 캐리지 잦은 왕복 방지

주의: distance_cm은 '팬 세기용 깊이'가 아니라 '캐리지에서 작업자까지 레일상 거리'.
      팬 세기(거리→PWM)는 별도 깊이 값으로 fan_control에서 처리한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Worker:
    """추적 중인 작업자 1명 (ID는 인식부 ByteTrack에서 부여)."""

    wid: int
    distance_cm: float = 0.0     # 캐리지↔작업자 레일상 거리 (가까울수록 이동 적음)
    cooled_total_s: float = 0.0  # 지금까지 누적 송풍받은 시간
    wait_s: float = 0.0          # 마지막 송풍 이후 대기 시간
    detected: bool = True        # 현재 프레임에 검출됐는지
    cooldown_until: float = 0.0  # 이 시각까지는 재선정 제외 (Cooldown)


def pick_target(
    workers: list[Worker],
    now: float,
    *,
    cooled_similar_s: float = 20.0,
    wait_similar_s: float = 20.0,
) -> Worker | None:
    """현재 송풍 대상 1명을 선정한다. 후보가 없으면 None.

    now: 현재 시각(초). Cooldown 판정에 사용.
    """
    # 1) 검출됐고 Cooldown이 끝난 후보만
    cands = [w for w in workers if w.detected and now >= w.cooldown_until]
    if not cands:
        return None
    if len(cands) == 1:
        return cands[0]

    # 2) 1순위: 누적 송풍시간 최소
    min_cooled = min(w.cooled_total_s for w in cands)
    tier1 = [w for w in cands if w.cooled_total_s - min_cooled <= cooled_similar_s]
    if len(tier1) == 1:
        return tier1[0]

    # 3) 2순위(동점): 대기시간 최대
    max_wait = max(w.wait_s for w in tier1)
    tier2 = [w for w in tier1 if max_wait - w.wait_s <= wait_similar_s]
    if len(tier2) == 1:
        return tier2[0]

    # 4) 3순위(또 동점): 캐리지에서 가까운 사람 → 캐리지 이동 최소(효율)
    return min(tier2, key=lambda w: w.distance_cm)
