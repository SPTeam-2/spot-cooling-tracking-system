"""가상 작업자별 누적 송풍시간 그래프 (통합부 / 김선형)

우리 우선순위 알고리즘으로 돌렸을 때, 다인원(가상 작업자 4명)이 10분 동안
누적 송풍을 얼마나 고르게 받는지를 작업자별 막대그래프로 보여준다.
(기획안 '자료 3. 다인원 우선순위 시뮬레이션 결과'용)

작업자는 레일을 따라 매초 조금씩 이동(현실 반영), 모두 재실(在室) 상태.
난수 시드 20개 평균(±표준편차)으로 특정 케이스 편향을 배제하고,
편차% = (최대-최소)/평균 × 100 은 시드별로 구한 뒤 평균낸다.

실행:  python3 scripts/worker_fairness.py
"""

from __future__ import annotations

import os
import random
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "control"))
from priority import Worker, pick_target

N = 4
BASE_X = [(i + 1) * 800 / (N + 1) for i in range(N)]   # 레일에 고르게 배치(mm)


def run(total_s=600, dwell=8, seed=0):
    rng = random.Random(seed)
    pos = list(BASE_X)
    workers = [Worker(wid=i) for i in range(N)]
    carriage = 400.0
    current = None
    dwell_left = 0
    for t in range(total_s):
        for i in range(N):
            pos[i] = max(50.0, min(750.0,
                     pos[i] + rng.gauss(0, 15) + 0.05 * (BASE_X[i] - pos[i])))
        if current is None or dwell_left <= 0:
            for w in workers:
                w.distance_cm = abs(pos[w.wid] - carriage)   # 캐리지 상대거리
            target = pick_target(workers, t)
            if target is not current:
                carriage = pos[target.wid]
                current = target
                dwell_left = dwell
        else:
            target = current
        if target is not None:
            carriage = pos[target.wid]
        for w in workers:
            if w is target:
                w.cooled_total_s += 1
                w.wait_s = 0
            else:
                w.wait_s += 1
        if target is not None:
            dwell_left -= 1
    return [w.cooled_total_s for w in workers]


def main(n_seeds=20):
    runs = [run(seed=s) for s in range(n_seeds)]
    per_worker_mean = [statistics.mean(r[i] for r in runs) for i in range(N)]
    per_worker_std = [statistics.pstdev(r[i] for r in runs) for i in range(N)]

    def spread(r):
        m = statistics.mean(r)
        return (max(r) - min(r)) / m * 100 if m else 0.0
    dev_pct = statistics.mean(spread(r) for r in runs)

    print(f"가상 작업자 {N}명, 10분(600초), 시드 {n_seeds}개 평균\n")
    for i in range(N):
        print(f"  작업자{i+1}: {per_worker_mean[i]:.0f} ± {per_worker_std[i]:.0f} 초")
    print(f"\n  평균 편차: {dev_pct:.1f}%  (목표 20% 이내)")

    _plot(per_worker_mean, per_worker_std, dev_pct)


def _plot(means, stds, dev_pct):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "AppleGothic"
    plt.rcParams["axes.unicode_minus"] = False

    labels = [f"작업자{i+1}" for i in range(N)]
    fig, ax = plt.subplots(figsize=(7, 4.6), dpi=150)
    bars = ax.bar(labels, means, yerr=stds, capsize=6, color="#2f7bbf", width=0.6)
    ax.bar_label(bars, fmt="%.0f", padding=4, fontsize=11)

    mean_all = statistics.mean(means)
    ax.axhline(mean_all, ls="--", lw=1.2, color="#444")
    ax.text(N - 0.5, mean_all, f" 평균 {mean_all:.0f}초", va="bottom", ha="right",
            fontsize=9, color="#444")

    ax.set_ylabel("10분간 누적 송풍시간 (초)")
    ax.set_title(f"다인원 우선순위 — 가상 작업자별 누적 송풍시간\n"
                 f"(작업자 4명, 시드 20개 평균 · 편차 {dev_pct:.1f}%, 목표 20% 이내)",
                 fontsize=11)
    ax.set_ylim(0, max(means) * 1.25)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    docs_dir = os.path.join(repo_root, "docs")
    os.makedirs(docs_dir, exist_ok=True)
    out = os.path.join(docs_dir, "worker_fairness.png")
    fig.savefig(out)
    print(f"\n그래프 저장: {out}")


if __name__ == "__main__":
    main()
