#!/bin/bash
# 목적: 펄스 방식 A 검증 — PWM으로 STEP을 내보내며 콜백으로 펄스를 세다가 STEPS개에서 정지 (1/8 스텝)
# 동작: 160 → 200Hz 단계 가속 → 250Hz 정속 → 200 → 160Hz 단계 감속, 가감속 펄스도 STEPS에 포함
#       회차마다 콜백 카운트 · 초과 스텝 · 정지 지연(N번째 펄스 → 정지 명령 완료) 출력
# 실행: bash hw/bash/carriage_pwm_count_test.bash   (먼저 pgrep pigpiod || sudo pigpiod)
# 확인: 축이나 풀리에 네임펜으로 표시 → 회차마다 표시가 제자리에 오는지 (SHUTTLE=1이면 짝수 회차 뒤)
#       초과 0~1스텝이고 표시가 매번 제자리면 A 진행 가능, 초과가 들쭉날쭉하면 정지 방식 보완 또는 B 유지 검토
# 주의: 리미트 확인 안 함. 캐리지를 레일 가운데 두고 시작 (1600스텝 = 1/8 기준 40mm)
# Python 버전: hw/py/stepper_pwm_count_check.py
python3 - <<'EOF'
import pigpio, threading, time

STEP, DIR, EN = 20, 21, 16
MICRO       = 8           # 1/8 스텝 (MS1·MS2 = 3.3V, MS3 미연결)
STEPS       = 1600        # 회차당 요청 스텝 (1/8 기준 1바퀴 = 40mm)
TOP_HZ      = 250         # 정속 주파수 (현장 안정값 200 또는 250)
RAMP        = [160, 200]  # 가속 단계(Hz), 감속은 역순
STAGE_STEPS = 40          # 가감속 단계 하나당 스텝
DIRECTION   = 1           # 0 또는 1
SHUTTLE     = 0           # 1이면 회차마다 방향을 바꿔 왕복
RUNS        = 3           # 반복 횟수
PAUSE       = 3           # 회차 사이 확인 시간(초)
SETTLE      = 0.5         # 정지 후 늦게 들어오는 펄스 집계 대기(초)

# (이 카운트에 도달하면, 이 주파수로)
plan = [(i * STAGE_STEPS, f) for i, f in enumerate(RAMP)]
plan.append((len(RAMP) * STAGE_STEPS, TOP_HZ))
plan += [(STEPS - (len(RAMP) - i) * STAGE_STEPS, f) for i, f in enumerate(reversed(RAMP))]
expected = sum((b - a) / f for (a, f), b in zip(plan, [c for c, _ in plan[1:]] + [STEPS]))

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
for p in (STEP, DIR, EN):
    pi.set_mode(p, pigpio.OUTPUT)

def pwm(freq):
    pi.set_PWM_frequency(STEP, freq)
    pi.set_PWM_dutycycle(STEP, 128)

st = {}
lock = threading.Lock()
def on_edge(gpio, level, tick):
    if level != 1:
        return
    with lock:
        st["count"] += 1
        if st["count"] == STEPS and st["stop_edge"] is None:
            pi.set_PWM_dutycycle(STEP, 0)
            st["stop_edge"], st["stop_done"] = tick, pi.get_current_tick()
            st["done"].set()
            return
        while st["idx"] < len(plan) and st["count"] >= plan[st["idx"]][0]:
            pwm(plan[st["idx"]][1])
            st["idx"] += 1

print(f"1/{MICRO} · 회차당 {STEPS}스텝 · 계획 {' → '.join(f'{f}Hz@{c}' for c, f in plan)} · 예상 {expected:.1f}초")
cb, over = None, []
try:
    pi.write(STEP, 0)
    pi.write(EN, 0)
    time.sleep(0.5)
    for r in range(1, RUNS + 1):
        d = DIRECTION if not SHUTTLE or r % 2 == 1 else 1 - DIRECTION
        pi.write(DIR, d)
        time.sleep(0.01)
        st = {"count": 0, "idx": 1, "stop_edge": None, "stop_done": None, "done": threading.Event()}
        cb = pi.callback(STEP, pigpio.RISING_EDGE, on_edge)
        t = time.time()
        pwm(plan[0][1])
        ok = st["done"].wait(expected + 3)
        if not ok:
            pi.set_PWM_dutycycle(STEP, 0)
        el = time.time() - t
        time.sleep(SETTLE)
        cb.cancel(); cb = None
        pi.write(STEP, 0)
        lat = pigpio.tickDiff(st["stop_edge"], st["stop_done"]) if st["stop_edge"] is not None else None
        over.append(st["count"] - STEPS)
        print(f"  {r}회차 DIR {d} {'완료' if ok else '시간 초과로 강제 정지'} ({el:.1f}초): "
              f"카운트 {st['count']} / 요청 {STEPS} · 초과 {st['count'] - STEPS:+d} · "
              f"정지 지연 {str(lat) + 'µs' if lat is not None else '측정 안 됨'} · 축 표시 확인")
        time.sleep(PAUSE)
except KeyboardInterrupt:
    print("중단됨")
finally:
    if cb is not None:
        cb.cancel()
    pi.set_PWM_dutycycle(STEP, 0)
    pi.write(STEP, 0)
    pi.write(EN, 1)
    pi.stop()
print(f"요약: 초과 스텝 {over}")
EOF