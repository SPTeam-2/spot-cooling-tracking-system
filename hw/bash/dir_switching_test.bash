#!/bin/bash
# 목적: 캐리지 왕복 반복으로 DIR 전환 확인 (1/8 스텝, 6.25mm/s)
# 동작: 가속 → 정속 RUN_S초 → 감속 → 잠깐 정지 → 방향 반전을 Ctrl+C까지 반복
# 실행: bash hw/bash/dir_switching_test.bash   (먼저 pgrep pigpiod || sudo pigpiod)
# 주의: 리미트 미연결 시 캐리지를 레일 가운데에 두고 시작, 편도 거리가 레일 안에 들어오게 RUN_S 조절
#       편도 거리(계산값) ≈ 6.25mm/s × RUN_S + 가감속 구간 약 5.4mm
python3 - <<'EOF'
import pigpio, time

STEP, DIR, EN = 20, 21, 16
MICRO      = 8            # 1/8 스텝
SPEED_MM_S = 6.25         # 250Hz (1/8 기준)
DIR_FIRST  = 0            # 먼저 갈 방향 (0 또는 1)
RUN_S      = 2            # 한 방향 정속 시간(초) → 6.25mm/s × 2초 + 가감속 ≈ 편도 18mm
RAMP_S     = 0.3          # 가속/감속 단계 하나당 시간
TURN_PAUSE = 0.2          # 방향 바꿀 때 잠깐 멈추는 시간

SPMM = 200 * MICRO / 40   # 1mm = 40스텝
RAMP = [160, 200, 250, 320]   # 가속 단계(Hz), 목표보다 낮은 값만 사용

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
for p in (STEP, DIR, EN):
    pi.set_mode(p, pigpio.OUTPUT)

def pwm(freq):
    real = pi.set_PWM_frequency(STEP, freq)
    pi.set_PWM_dutycycle(STEP, 128)
    return real

top = int(SPEED_MM_S * SPMM)
ramp = [f for f in RAMP if f < top]

def ramp_down():
    for f in reversed(ramp):
        pwm(f)
        time.sleep(RAMP_S)
    pi.set_PWM_dutycycle(STEP, 0)
    pi.write(STEP, 0)

try:
    pi.write(STEP, 0)
    pi.write(EN, 0)
    time.sleep(0.05)
    direction = DIR_FIRST
    while True:
        pi.write(DIR, direction)
        time.sleep(0.01)
        for f in ramp:                      # 천천히 출발
            pwm(f)
            time.sleep(RAMP_S)
        real = pwm(top)                     # 목표 속도
        print(f"DIR {direction} · {real / SPMM:.1f}mm/s ({real}Hz)로 {RUN_S}초 회전")
        time.sleep(RUN_S)
        ramp_down()                         # 부드럽게 멈춤
        time.sleep(TURN_PAUSE)
        direction = 1 - direction           # 방향 뒤집기
except KeyboardInterrupt:
    print("감속 후 정지")
    ramp_down()
finally:
    pi.set_PWM_dutycycle(STEP, 0)
    pi.write(STEP, 0)
    pi.write(EN, 1)
    pi.stop()
EOF