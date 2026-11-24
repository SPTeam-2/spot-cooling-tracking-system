#!/bin/bash
# 목적: 정방향/역방향 속도를 따로 줘서 왕복 부드러움 비교 (1/8 스텝)
# 동작: 가속 → 정속 → 감속 → 잠깐 정지 → 방향 반전을 Ctrl+C까지 반복
# 실행: bash hw/bash/dir_speed_compare.bash   (먼저 pgrep pigpiod || sudo pigpiod)
# 주의: 리미트 미연결 시 캐리지를 레일 가운데에 두고 시작
#       양방향 편도 거리를 같게 맞춰서 속도가 달라도 한쪽으로 밀려가지 않음
#       PWM 주파수는 정해진 값만 가능: 160 200 250 320 400 500 800 1000 1600 2000Hz
#       (mm/s = Hz / 40 → 4, 5, 6.25, 8, 10, 12.5, 20, 25, 40, 50)
python3 - <<'EOF'
import pigpio, time

STEP, DIR, EN = 20, 21, 16
MICRO      = 8
SPEED_MM_S = {0: 6.25, 1: 8.0}   # 방향별 속도 (0=정방향, 1=역방향)
DIR_FIRST  = 0
ONE_WAY_MM = 18           # 편도 거리 (가감속 포함)
RAMP_S     = 0.3          # 가속/감속 단계 하나당 시간
TURN_PAUSE = 0.2

SPMM = 200 * MICRO / 40   # 1mm = 40스텝
RAMP = [160, 200, 250, 320, 400, 500, 800, 1000]

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
for p in (STEP, DIR, EN):
    pi.set_mode(p, pigpio.OUTPUT)

def pwm(freq):
    real = pi.set_PWM_frequency(STEP, freq)
    pi.set_PWM_dutycycle(STEP, 128)
    return real

def stop_pulses():
    pi.set_PWM_dutycycle(STEP, 0)
    pi.write(STEP, 0)

def plan(direction):
    top = int(SPEED_MM_S[direction] * SPMM)
    ramp = [f for f in RAMP if f < top]
    ramp_mm = 2 * sum(ramp) * RAMP_S / SPMM
    return top, ramp, ramp_mm

def ramp_down(ramp):
    for f in reversed(ramp):
        pwm(f)
        time.sleep(RAMP_S)
    stop_pulses()

ramp = []
try:
    pi.write(STEP, 0)
    pi.write(EN, 0)
    time.sleep(0.05)
    direction = DIR_FIRST
    while True:
        top, ramp, ramp_mm = plan(direction)
        pi.write(DIR, direction)
        time.sleep(0.01)
        for f in ramp:
            pwm(f)
            time.sleep(RAMP_S)
        real = pwm(top)
        speed = real / SPMM
        run_s = max(0.0, (ONE_WAY_MM - ramp_mm) / speed)
        note = "" if real == top else f"  ※ {top}Hz 불가 → {real}Hz로 바뀜"
        print(f"DIR {direction} · {speed:.2f}mm/s ({real}Hz)로 {run_s:.1f}초{note}")
        time.sleep(run_s)
        ramp_down(ramp)
        time.sleep(TURN_PAUSE)
        direction = 1 - direction
except KeyboardInterrupt:
    print("감속 후 정지")
    ramp_down(ramp)
finally:
    stop_pulses()
    pi.write(EN, 1)
    pi.stop()
EOF