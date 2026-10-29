"""우선순위 알고리즘 시뮬레이션 (하드웨어 없이 성능 측정)

가상 작업자 N명을 두고 우선순위 알고리즘으로 송풍 대상을 번갈아 선정하며,
"인당 누적 송풍시간 편차 %"를 계산한다. (보고서 성능지표: 편차 20% 이내 목표)

실행:  python3 src/control/simulate_priority.py
"""

from __future__ import annotations

import random
import statistics

from priority import Worker, pick_target


def run_sim(
    n_workers: int,
    total_s: int = 600,
    min_dwell_s: float = 8.0,
    cooldown_s: float = 5.0,
    seed: int = 0,
) -> dict:
    """N명이 통로에 있는 상황을 total_s초 동안 1초 단위로 시뮬레이션.

    반환: 인당 누적 송풍시간, 편차 지표.
    """
    rng = random.Random(seed)
    # 작업자별 거리는 서로 다르게(1~2m) 두어 거리 기준도 동작하게 함
    workers = [
        Worker(wid=i, distance_cm=rng.uniform(40, 200))
        for i in range(n_workers)
    ]

    current: Worker | None = None
    dwell_left = 0.0

    for t in range(total_s):
        # 최소 유지시간이 남아있으면 현재 대상 유지, 아니면 재선정
        if current is None or dwell_left <= 0:
            target = pick_target(workers, now=t)
            if target is not current and target is not None:
                current = target
                dwell_left = min_dwell_s
        else:
            target = current

        for w in workers:
            if w is target:
                w.cooled_total_s += 1      # 송풍 1초
                w.wait_s = 0
            else:
                w.wait_s += 1              # 나머지는 대기 누적

        if target is not None:
            dwell_left -= 1
            # 유지시간 끝나면 그 대상 Cooldown 부여 (바로 재선정 방지)
            if dwell_left <= 0:
                target.cooldown_until = t + cooldown_s

    cooled = [w.cooled_total_s for w in workers]
    mean = statistics.mean(cooled)
    # 편차% = (최대-최소)/평균 × 100  (보고서 지표 정의와 맞춤)
    spread_pct = (max(cooled) - min(cooled)) / mean * 100 if mean else 0.0
    return {
        "n": n_workers,
        "cooled_s": [round(c, 1) for c in cooled],
        "mean_s": round(mean, 1),
        "spread_pct": round(spread_pct, 1),
    }


def main() -> None:
    print("다중 작업자 송풍 우선순위 시뮬레이션 (600초, 최소유지 8s, Cooldown 5s)\n")
    print(f"{'인원':>4} | {'인당 송풍시간(s)':<28} | {'평균':>6} | {'편차%':>6}")
    print("-" * 60)
    worst = 0.0
    for n in range(2, 6):
        r = run_sim(n)
        print(
            f"{r['n']:>4} | {str(r['cooled_s']):<28} | "
            f"{r['mean_s']:>6} | {r['spread_pct']:>5}%"
        )
        worst = max(worst, r["spread_pct"])
    print("-" * 60)
    print(f"2~5명 최대 편차: {worst:.1f}%  (목표: 20% 이내)")
    print("→ 결과를 경진대회 보고서 '다인원 우선순위 알고리즘' 편차% 칸에 기입")


if __name__ == "__main__":
    main()
