"""송풍 분배 전략 비교 — 작업자 3명(거리 다름)에서 두 방식의 공정성 편차% 대조.

(A) 최근접만(베이스라인) vs (B) 우선순위(누적→대기→거리). 콘솔 표 + 막대그래프(PNG).

실행:  python3 scripts/compare_strategies.py
"""

from __future__ import annotations

import os
import statistics
import sys

# 로직 모듈(src/control/priority.py) import 경로 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "control"))
from priority import Worker, pick_target


def _spread_pct(vals: list[float]) -> float:
    """편차% = (최대-최소)/평균 × 100.  0에 가까울수록 공정."""
    m = statistics.mean(vals)
    return (max(vals) - min(vals)) / m * 100 if m else 0.0


def run(strategy: str, distances_mm, total_s=600, min_dwell_s=8.0, cooldown_s=5.0):
    """고정 거리 상황을 1초 단위로 시뮬레이션. strategy: "nearest" | "priority"."""
    workers = [Worker(wid=i, distance_mm=d) for i, d in enumerate(distances_mm)]
    current: Worker | None = None
    dwell_left = 0.0

    for t in range(total_s):
        if current is None or dwell_left <= 0:
            if strategy == "nearest":
                # 단순 방식: 검출된 사람 중 가장 가까운 사람 (누적/대기 무시)
                cands = [w for w in workers if w.detected]
                target = min(cands, key=lambda w: w.distance_mm) if cands else None
            else:
                target = pick_target(workers, now=t)
            if target is not None and target is not current:
                current = target
                dwell_left = min_dwell_s
        else:
            target = current

        for w in workers:
            if w is target:
                w.cooled_total_s += 1
                w.wait_s = 0
            else:
                w.wait_s += 1

        if target is not None:
            dwell_left -= 1
            if dwell_left <= 0:
                # 우리 방식만 Cooldown 부여(재선정 분산). 단순 방식은 바로 최근접 복귀.
                if strategy == "priority":
                    target.cooldown_until = t + cooldown_s

    return [w.cooled_total_s for w in workers]


def main() -> None:
    distances = [600, 1100, 1800]  # mm — 가깝/중간/먼 작업자 (레일상 거리)
    labels = [f"작업자{i+1}\n({d}mm)" for i, d in enumerate(distances)]

    nearest = run("nearest", distances)
    priority = run("priority", distances)

    print("상황: 작업자 3명 (레일상 거리 600 / 1100 / 1800 mm), 10분(600s) 송풍\n")
    print(f"{'방식':<20} | {'인당 송풍시간(s)':<24} | {'편차%':>6}")
    print("-" * 58)
    print(f"{'가장 가까운 사람만':<18} | {str([round(x) for x in nearest]):<24} | {_spread_pct(nearest):>5.1f}%")
    print(f"{'우선순위 알고리즘':<18} | {str([round(x) for x in priority]):<24} | {_spread_pct(priority):>5.1f}%")
    print("-" * 58)
    print("→ 단순 최근접은 먼 작업자가 송풍을 거의 못 받음(편차 큼),")
    print("  우선순위 알고리즘은 세 명에게 고르게 분배(편차 작음).")

    _plot(labels, nearest, priority)


def _plot(labels, nearest, priority) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from _plot_style import set_korean_font
    set_korean_font(plt)

    x = range(len(labels))
    w = 0.38
    fig, ax = plt.subplots(figsize=(7.2, 4.6), dpi=150)

    b1 = ax.bar([i - w / 2 for i in x], nearest, w,
                label=f"가장 가까운 사람만 (편차 {_spread_pct(nearest):.0f}%)",
                color="#c9513b")
    b2 = ax.bar([i + w / 2 for i in x], priority, w,
                label=f"우선순위 알고리즘 (편차 {_spread_pct(priority):.0f}%)",
                color="#2f7bbf")

    ax.set_ylabel("10분간 받은 송풍시간 (초)")
    ax.set_title("송풍 분배 방식 비교 — 작업자 3명")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels)
    ax.legend(loc="upper right", fontsize=9)
    ax.bar_label(b1, fmt="%d", padding=2, fontsize=9)
    ax.bar_label(b2, fmt="%d", padding=2, fontsize=9)
    ax.set_ylim(0, max(nearest + priority) * 1.18)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()

    # 저장 위치는 실행 디렉터리와 무관하게 레포 루트의 docs/로 고정
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    docs_dir = os.path.join(repo_root, "docs")
    os.makedirs(docs_dir, exist_ok=True)
    out = os.path.join(docs_dir, "strategy_comparison.png")
    fig.savefig(out)
    print(f"\n그래프 저장: {out}")


if __name__ == "__main__":
    main()
