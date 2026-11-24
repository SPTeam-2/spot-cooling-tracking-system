#!/bin/bash
# 목적: 캐리지 한 방향 연속 회전 확인 (1/8 스텝, 6.25mm/s)
# 동작: 160 → 200Hz 단계 가속 → 250Hz 정속 유지 → Ctrl+C 시 단계 감속 후 정지
# 실행: bash hw/bash/carriage_driving_test.bash   (먼저 pgrep pigpiod || sudo pigpiod)
# 주의: 리미트 미연결 시 캐리지를 레일 가운데에 두고 시작, 끝에 닿기 전에 Ctrl+C
#       방향은 DIRECTION 값으로 바꿈 (0 또는 1)

python3 - <<'EOF'
import pigpio, time

STEP, DIR, EN = 20, 21, 16
MICRO      = 8            # 1/8 스텝
SPEED_MM_S = 8         # 250Hz (1/8 기준)
DIRECTION  = 1            # 0 또는 1
RAMP_S     = 0.3          # 가속/감속 단계 하나당 시간(초)

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
try:
    pi.write(STEP, 0)
    pi.write(DIR, DIRECTION)
    pi.write(EN, 0)
    time.sleep(0.05)
    for f in RAMP:
        if f < top:
            pwm(f)
            time.sleep(RAMP_S)
    real = pwm(top)
    print(f"DIR {DIRECTION} · {real / SPMM:.1f}mm/s ({real}Hz)로 계속 회전 중 (Ctrl+C로 정지)")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("감속 후 정지")
    for f in reversed(RAMP):
        if f < top:
            pwm(f)
            time.sleep(RAMP_S)
finally:
    pi.set_PWM_dutycycle(STEP, 0)
    pi.write(STEP, 0)
    pi.write(EN, 1)
    pi.stop()
EOF