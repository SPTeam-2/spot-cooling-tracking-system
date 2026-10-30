"""현실 시나리오 기반 송풍 분배 전략 비교 (통합부 / 김선형)

작업자 4명이 레일을 따라 서로 다른 위치에서 일하며 매초 조금씩 이동하고,
가끔 자리를 비웠다 복귀한다(휴식·자재 이동). 이 현실적 상황에서 세 전략을 비교:

  (1) 단순 최근접  : 매번 캐리지에서 가장 가까운 사람 (분배 개념 없음)
  (2) 라운드로빈   : 정해진 순서대로 공평하게 돌림 (거리 완전 무시)
  (3) 우선순위     : 누적 송풍 적은 사람 우선 + 비슷하면 가까운 사람 (우리 방식)

세 지표로 평가(모두 "낮을수록 좋음"):
  - 공정성(편차%)     : 재실(在室)시간 대비 송풍시간 비율의 편차 → 0에 가까울수록 공정
  - 캐리지 이동거리(mm): 캐리지가 레일을 왕복한 총 거리 → 작을수록 효율(모터부하·전력·소음↓)
  - 최대 대기시간(s)   : 자리에 있는데 송풍 못 받고 가장 오래 기다린 시간

난수 시드 여러 개로 반복해 평균±표준편차로 집계(특정 케이스 편향 방지).

실행:  python3 scripts/scenario_sim.py
"""

from __future__ import annotations

import os
import random
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "control"))
from priority import Worker, pick_target

BASE_X = [100, 300, 500, 700]   # 레일 위 작업자 기본 위치(mm, 800mm 레일)
RAIL_START = 400.0              # 캐리지 초기 위치(레일 중앙)


def run(strategy, total_s=1200, min_dwell_s=8, seed=0):
    rng = random.Random(seed)
    n = len(BASE_X)
    workers = [Worker(wid=i) for i in range(n)]
    rail_x = [float(BASE_X[i]) for i in range(n)]
    absent_left = [0] * n
    present_s = [0] * n
    max_wait = [0] * n

    carriage = RAIL_START
    travel = 0.0
    current = None
    dwell_left = 0
    last_id = None

    for t in range(total_s):
        # 1) 이동 + 입출입
        for i, w in enumerate(workers):
            if absent_left[i] > 0:
                absent_left[i] -= 1
                w.detected = absent_left[i] == 0
            elif rng.random() < 0.0015:
                absent_left[i] = rng.randint(20, 80)
                w.detected = False
            rail_x[i] = max(50.0, min(750.0,
                        rail_x[i] + rng.gauss(0, 15) + 0.05 * (BASE_X[i] - rail_x[i])))

        # 2) 대상 선정 (유지시간 끝났거나 대상이 사라지면 재선정)
        if current is None or dwell_left <= 0 or not current.detected:
            cands = [w for w in workers if w.detected]
            if not cands:
                target = None
            elif strategy == "단순 최근접":
                target = min(cands, key=lambda w: abs(rail_x[w.wid] - carriage))
            elif strategy == "라운드로빈":
                start = (last_id + 1) if last_id is not None else 0
                target = next((workers[(start + k) % n] for k in range(n)
                               if workers[(start + k) % n].detected), None)
            else:  # 우선순위(우리): 거리 = 캐리지에서의 레일상 거리(이동 최소)
                for w in workers:
                    w.distance_cm = abs(rail_x[w.wid] - carriage)
                target = pick_target(workers, t)

            if target is not current:
                if target is not None:
                    travel += abs(rail_x[target.wid] - carriage)
                    carriage = rail_x[target.wid]
                    last_id = target.wid
                current = target
                dwell_left = min_dwell_s
        else:
            target = current

        # 3) 송풍/대기/재실 누적, 캐리지는 대상 따라 미세 추종
        for w in workers:
            if w.detected:
                present_s[w.wid] += 1
            if w is target and target is not None:
                w.cooled_total_s += 1
                w.wait_s = 0
            elif w.detected:
                w.wait_s += 1
                max_wait[w.wid] = max(max_wait[w.wid], w.wait_s)

        if target is not None:
            carriage = rail_x[target.wid]
            dwell_left -= 1

    cov = [workers[i].cooled_total_s / present_s[i] if present_s[i] else 0
           for i in range(n)]
    mean_cov = statistics.mean(cov)
    fairness_pct = (max(cov) - min(cov)) / mean_cov * 100 if mean_cov else 0.0
    return {"fairness_pct": fairness_pct, "travel_mm": travel, "max_wait_s": max(max_wait)}


STRATEGIES = ["단순 최근접", "라운드로빈", "우선순위(우리)"]


def main(total_s=1200, min_dwell_s=8, n_seeds=20):
    print(f"현실 시나리오: 작업자 4명, 이동+입출입, {total_s}초, 시드 {n_seeds}개 평균\n")
    agg = {}
    for name in STRATEGIES:
        runs = [run(name, total_s, min_dwell_s, seed=s) for s in range(n_seeds)]
        agg[name] = {k: (statistics.mean(r[k] for r in runs),
                         statistics.pstdev(r[k] for r in runs))
                     for k in ("fairness_pct", "travel_mm", "max_wait_s")}

    print(f"{'전략':<16}| {'공정성 편차%':>11} | {'캐리지 이동(mm)':>14} | {'최대 대기(s)':>11}")
    print("-" * 64)
    for name, m in agg.items():
        print(f"{name:<15} | {m['fairness_pct'][0]:>7.1f} ±{m['fairness_pct'][1]:>3.1f} | "
              f"{m['travel_mm'][0]:>9.0f} ±{m['travel_mm'][1]:>4.0f} | "
              f"{m['max_wait_s'][0]:>6.0f} ±{m['max_wait_s'][1]:>3.0f}")
    print("-" * 64)
    print("단순 최근접: 이동은 적지만 가까운 사람만 지켜 공정성·대기 붕괴(사실상 탈락).")
    print("우리 방식: 라운드로빈 수준 공정성을 유지하며 캐리지 이동을 크게 줄임(효율).")

    _plot(agg, total_s, n_seeds)
    return agg


def _plot(agg, total_s, n_seeds):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "AppleGothic"
    plt.rcParams["axes.unicode_minus"] = False

    names = list(agg.keys())
    colors = ["#c9513b", "#8a8f99", "#2f7bbf"]  # 최근접=빨강, 라운드로빈=회색, 우리=파랑
    panels = [
        ("fairness_pct", "공정성 — 송풍시간 편차 (%)", "낮을수록 공정", 20),
        ("travel_mm", "효율 — 캐리지 이동거리 (mm)", "낮을수록 효율적", None),
        ("max_wait_s", "최대 대기시간 (초)", "낮을수록 덜 방치", None),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.9), dpi=150)
    for ax, (key, title, sub, target) in zip(axes, panels):
        means = [agg[n][key][0] for n in names]
        stds = [agg[n][key][1] for n in names]
        bars = ax.bar(names, means, yerr=stds, capsize=5, color=colors)
        ax.set_title(f"{title}\n({sub})", fontsize=11)
        ax.bar_label(bars, fmt="%.0f", padding=3, fontsize=10)
        ax.set_ylim(0, max(means) * 1.25)
        ax.grid(axis="y", alpha=0.25)
        ax.tick_params(axis="x", labelsize=9)
        if target is not None:   # 공정성 목표선
            ax.axhline(target, ls="--", lw=1.2, color="#444")
            ax.text(len(names) - 0.4, target, f" 목표 {target}%", va="bottom",
                    ha="right", fontsize=8, color="#444")

    fig.suptitle(
        f"현실 시나리오 송풍 분배 비교 — 작업자 4명, 이동+입출입 "
        f"({total_s // 60}분, 시드 {n_seeds}개 평균±표준편차)",
        fontsize=12,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    docs_dir = os.path.join(repo_root, "docs")
    os.makedirs(docs_dir, exist_ok=True)
    out = os.path.join(docs_dir, "scenario_comparison.png")
    fig.savefig(out)
    print(f"\n그래프 저장: {out}")


if __name__ == "__main__":
    main()
