"""PWM 펄스 + 콜백 카운트 정지 점검 (펄스 방식 A 검증용).

목적: STEP을 pigpio PWM으로 내보내면서 콜백으로 펄스 수를 세다가 N개에서 멈추는 방식이
      스텝 수를 정확히 맞추는지 확인한다. function_stepper.py를 A 방식으로 바꾸기 전 확인 단계.
      확인할 것: (1) 콜백 카운트가 요청 스텝과 같은지 (2) 정지 명령 뒤 더 나간 펄스(초과 스텝)
      (3) 정지 지연(N번째 펄스 → 정지 명령 완료) (4) 축 표시가 매 회차 제자리로 오는지
내용: STEP 핀 상승 에지를 pigpio 콜백으로 센다. 가속·감속 구간 펄스도 요청 스텝에 포함한다.
      160 → 200Hz 단계 가속 → 250Hz 정속 → 200 → 160Hz 단계 감속, 단계 전환·정지는 콜백 안에서 카운트 기준으로 한다.
흐름: 핀 출력 설정 → 콜백 등록 → DIR·EN 설정 → PWM 시작 → 카운트 기준 단계 전환 → N개에서 duty 0
      → 잠시 대기 후 늦게 들어온 펄스까지 집계 → 결과 출력 → (반복) → EN 해제.

주의: 콜백은 pigpiod 알림을 받아 Python 스레드에서 처리하므로 정지가 늦을 수 있다. 그 크기를 재는 스크립트다.
      리미트는 확인하지 않는다. 캐리지를 레일 가운데 두고, 1600스텝(1/8 기준 40mm)보다 여유가 있는지 확인 후 실행.
      출력 핀 콜백 카운트가 실제 출력 펄스와 같다는 전제는 축 표시(실물)로 함께 확인한다.

예)  python3 hw/py/stp_pwm_count_check.py                      # 1600스텝, 250Hz, 가감속 포함, 3회
     python3 hw/py/stp_pwm_count_check.py --shuttle --runs 4    # 왕복 반복 (표시 위치 복귀 확인)
     python3 hw/py/stp_pwm_count_check.py --no-ramp --hz 200    # 가감속 없이 200Hz만
"""

from __future__ import annotations

import argparse
import threading
import time

from _hw_common import PWM_ALLOWED_HZ, connect, load_pins, ramp_steps, steps_per_rev


def build_plan(total: int, top_hz: int, ramp_hz: list[int], stage_steps: int) -> list[tuple[int, int]]:
    """(이 카운트에 도달하면, 이 주파수로) 목록. 가감속 구간 스텝도 total 안에 포함한다.

    total이 가감속에 필요한 스텝보다 작으면 단계 수를 줄인다 (짧은 이동은 낮은 주파수만 사용).
    """
    while ramp_hz and len(ramp_hz) * 2 * stage_steps >= total:
        ramp_hz = ramp_hz[:-1]           # 높은 가속 단계부터 뺀다
    plan = []
    count = 0
    for f in ramp_hz:                    # 가속: 0, s, 2s ... 에서 전환
        plan.append((count, f))
        count += stage_steps
    plan.append((count, top_hz))         # 정속 시작
    decel_start = total - len(ramp_hz) * stage_steps
    for i, f in enumerate(reversed(ramp_hz)):
        plan.append((decel_start + i * stage_steps, f))
    return plan


class CountRun:
    """한 회차: 콜백으로 상승 에지를 세며 계획대로 주파수를 바꾸고 total에서 duty 0."""

    def __init__(self, pi, pigpio, step_pin: int, total: int, plan: list[tuple[int, int]]):
        self.pi, self.pigpio, self.step = pi, pigpio, step_pin
        self.total = total
        self.plan = plan
        self.next_idx = 1                # plan[0]은 시작 시 적용
        self.count = 0
        self.ticks: list[int] = []       # 에지 tick(µs), 간격·실제 주파수 계산용
        self.freq_at: list[int] = []     # 각 에지 직전 적용 주파수
        self.current_hz = plan[0][1]
        self.stop_edge_tick: int | None = None
        self.stop_done_tick: int | None = None
        self.done = threading.Event()
        self.lock = threading.Lock()

    def set_hz(self, hz: int) -> None:
        self.pi.set_PWM_frequency(self.step, hz)
        self.pi.set_PWM_dutycycle(self.step, 128)
        self.current_hz = hz

    def on_edge(self, gpio, level, tick) -> None:
        if level != 1:
            return
        with self.lock:
            self.count += 1
            self.ticks.append(tick)
            self.freq_at.append(self.current_hz)
            if self.count == self.total and self.stop_edge_tick is None:
                self.pi.set_PWM_dutycycle(self.step, 0)
                self.stop_edge_tick = tick
                self.stop_done_tick = self.pi.get_current_tick()
                self.done.set()
                return
            while self.next_idx < len(self.plan) and self.count >= self.plan[self.next_idx][0]:
                self.set_hz(self.plan[self.next_idx][1])
                self.next_idx += 1

    def start(self) -> None:
        self.set_hz(self.plan[0][1])

    def force_stop(self) -> None:
        self.pi.set_PWM_dutycycle(self.step, 0)


def summarize(run: CountRun, pigpio, settle_s: float) -> dict:
    with run.lock:
        count = run.count
        ticks = list(run.ticks)
        freqs = list(run.freq_at)
    per_hz: dict[int, list[int]] = {}
    for i in range(1, len(ticks)):
        if freqs[i] == freqs[i - 1]:     # 주파수 전환 직후 간격은 제외
            per_hz.setdefault(freqs[i], []).append(pigpio.tickDiff(ticks[i - 1], ticks[i]))
    hz_stats = {}
    for hz, iv in sorted(per_hz.items()):
        mean = sum(iv) / len(iv)
        hz_stats[hz] = (len(iv) + 1, 1e6 / mean, min(iv), max(iv))
    latency = None
    if run.stop_edge_tick is not None and run.stop_done_tick is not None:
        latency = pigpio.tickDiff(run.stop_edge_tick, run.stop_done_tick)
    return {"count": count, "overshoot": count - run.total, "latency_us": latency, "hz": hz_stats,
            "settle_s": settle_s}


def main() -> None:
    ap = argparse.ArgumentParser(description="PWM 펄스 + 콜백 카운트 정지 점검 (방식 A)")
    ap.add_argument("--micro", type=int, default=8, help="A4988 마이크로스텝 (현재 배선 1/8 → 8)")
    ap.add_argument("--steps", type=int, default=0, help="회차당 요청 스텝. 0이면 1바퀴(200 × micro)")
    ap.add_argument("--hz", type=int, default=250, help="정속 주파수 (현장 안정값 200 또는 250)")
    ap.add_argument("--stage-steps", type=int, default=40, help="가속·감속 단계 하나당 스텝 수")
    ap.add_argument("--no-ramp", action="store_true", help="가감속 없이 --hz로만 출력")
    ap.add_argument("--dir", type=int, choices=(0, 1), default=1, help="DIR 출력 (0 또는 1)")
    ap.add_argument("--shuttle", action="store_true", help="회차마다 방향을 바꿔 왕복")
    ap.add_argument("--runs", type=int, default=3, help="반복 횟수")
    ap.add_argument("--pause", type=float, default=3, help="회차 사이 확인 시간(초)")
    ap.add_argument("--settle", type=float, default=0.5, help="정지 후 늦은 펄스 집계 대기(초)")
    ap.add_argument("--timeout-margin", type=float, default=3, help="예상 시간 + 이 값(초) 안에 끝나지 않으면 강제 정지")
    args = ap.parse_args()

    if args.hz not in PWM_ALLOWED_HZ:
        raise SystemExit(f"--hz는 pigpio 허용 주파수 중 하나여야 함: {PWM_ALLOWED_HZ}")
    total = args.steps or steps_per_rev(args.micro)
    if total <= 0 or args.stage_steps <= 0:
        raise SystemExit("--steps·--stage-steps는 양수")
    ramp = [] if args.no_ramp else ramp_steps(args.hz)
    plan = build_plan(total, args.hz, ramp, args.stage_steps)

    pins = load_pins()["stepper"]
    step, dirp, en = pins["step"], pins["dir"], pins["enable"]
    pigpio, pi = connect()

    expected_s = 0.0
    edges = [c for c, _ in plan] + [total]
    for (c, f), nxt in zip(plan, edges[1:]):
        expected_s += (nxt - c) / f
    print(f"1/{args.micro} · 회차당 {total}스텝 ({total / steps_per_rev(args.micro):g}바퀴) · "
          f"계획 {' → '.join(f'{f}Hz@{c}' for c, f in plan)} · 예상 {expected_s:.1f}초")

    cb = None
    results = []
    try:
        for p in (step, dirp, en):
            pi.set_mode(p, pigpio.OUTPUT)
        pi.write(step, 0)
        pi.write(en, 0)
        time.sleep(0.5)
        for r in range(1, args.runs + 1):
            direction = args.dir if not args.shuttle or r % 2 == 1 else 1 - args.dir
            pi.write(dirp, direction)
            time.sleep(0.01)
            run = CountRun(pi, pigpio, step, total, plan)
            cb = pi.callback(step, pigpio.RISING_EDGE, run.on_edge)
            t0 = time.time()
            run.start()
            finished = run.done.wait(expected_s + args.timeout_margin)
            if not finished:
                run.force_stop()
            elapsed = time.time() - t0
            time.sleep(args.settle)      # 정지 뒤에 도착하는 알림까지 집계
            cb.cancel()
            cb = None
            pi.write(step, 0)
            s = summarize(run, pigpio, args.settle)
            results.append(s)
            state = "완료" if finished else "시간 초과로 강제 정지"
            lat = f"{s['latency_us']}µs" if s["latency_us"] is not None else "측정 안 됨"
            print(f"  {r}회차 DIR {direction} {state} ({elapsed:.1f}초): 카운트 {s['count']} / 요청 {total} · "
                  f"초과 {s['overshoot']:+d} · 정지 지연 {lat}")
            for hz, (n, real, mn, mx) in s["hz"].items():
                print(f"      {hz}Hz 구간 {n}펄스 · 측정 {real:.1f}Hz · 간격 {mn}~{mx}µs")
            print("      축 표시 위치 확인" + (" (왕복이면 짝수 회차 뒤 처음 자리)" if args.shuttle else ""))
            time.sleep(args.pause)
    except KeyboardInterrupt:
        print("중단됨")
    finally:
        if cb is not None:
            cb.cancel()
        pi.set_PWM_dutycycle(step, 0)
        pi.write(step, 0)
        pi.write(en, 1)
        pi.stop()

    if results:
        over = [s["overshoot"] for s in results]
        lats = [s["latency_us"] for s in results if s["latency_us"] is not None]
        print(f"요약: 초과 스텝 {over}" + (f" · 정지 지연 최대 {max(lats)}µs" if lats else ""))
        print("판단 기준(제안): 초과 0~1스텝이고 축 표시가 매번 제자리면 A 진행 가능, "
              "초과가 들쭉날쭉하면 정지 방식 보완 또는 B 유지 검토")


if __name__ == "__main__":
    main()