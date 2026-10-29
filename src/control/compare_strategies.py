"""송풍 분배 전략 비교 시뮬레이션 
같은 상황(작업자 3명, 서로 다른 거리)에서 두 방식을 비교한다.

  (A) 가장 가까운 사람만 송풍  — 단순 방식(베이스라인)
  (B) 우선순위 알고리즘        — 누적송풍→대기→거리 (우리 방식)

목적: "단순 추적/최근접"이 아니라 '다인원 공정 분배'가 핵심 기여임을 수치로 보여줌.
결과를 콘솔 표 + 막대그래프(PNG)로 출력 → 캡스톤 신청서 근거자료로 사용.

실행:  python3 src/control/compare_strategies.py
"""

from __future__ import annotations

import statistics

from priority import Worker, pick_target


def _spread_pct(vals: list[float]) -> float:
    """편차% = (최대-최소)/평균 × 100.  0에 가까울수록 공정."""
    m = statistics.mean(vals)
    return (max(vals) - min(vals)) / m * 100 if m else 0.0


def run(strategy: str, distances_cm, total_s=600, min_dwell_s=8.0, cooldown_s=5.0):
    """3명 고정 거리 상황을 total_s초간 1초 단위로 시뮬레이션.

    strategy="nearest" : 매 순간 가장 가까운 사람 (공정성 고려 X)
    strategy="priority": 우리 우선순위 알고리즘
    반환: 인당 누적 송풍시간 리스트
    """
    workers = [Worker(wid=i, distance_cm=d) for i, d in enumerate(distances_cm)]
    current: Worker | None = None
    dwell_left = 0.0

    for t in range(total_s):
        if current is None or dwell_left <= 0:
            if strategy == "nearest":
                # 단순 방식: 검출된 사람 중 가장 가까운 사람 (누적/대기 무시)
                cands = [w for w in workers if w.detected]
                target = min(cands, key=lambda w: w.distance_cm) if cands else None
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
    distances = [60, 110, 180]  # cm — 가깝/중간/먼 작업자
    labels = [f"작업자{i+1}\n({d}cm)" for i, d in enumerate(distances)]

    nearest = run("nearest", distances)
    priority = run("priority", distances)

    print("상황: 작업자 3명 (거리 60 / 110 / 180 cm), 10분(600s) 송풍\n")
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
    plt.rcParams["font.family"] = "AppleGothic"
    plt.rcParams["axes.unicode_minus"] = False

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

    import os
    # 저장 위치는 실행 디렉터리와 무관하게 레포 루트의 docs/로 고정
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    docs_dir = os.path.join(repo_root, "docs")
    os.makedirs(docs_dir, exist_ok=True)
    out = os.path.join(docs_dir, "strategy_comparison.png")
    fig.savefig(out)
    print(f"\n그래프 저장: {out}")


if __name__ == "__main__":
    main()
