#!/bin/bash
# 목적: 안정 속도 2단계(200Hz, 250Hz)에서 캐리지가 매끄럽게 도는지 확인 (1/8 스텝, PWM 방식)
# 동작: 160Hz에서 출발 → 200Hz(5mm/s) SECONDS초 → 250Hz(6.25mm/s) SECONDS초 → 단계 감속 정지
# 실행: bash hw/bash/carriage_speed_sweep_test.bash   (먼저 pgrep pigpiod || sudo pigpiod)
# 근거: 2026-11-23 현장 점검에서 200Hz, 250Hz에서만 캐리지가 안정적으로 움직임
#       pigpio 파형(wave) 방식은 이 Pi에서 STEP 펄스가 나가지 않아 PWM 방식 사용
# 주의: 리미트 미연결 시 캐리지를 레일 끝에서 떨어뜨려 시작
#       이동 거리(계산값) ≈ (5 + 6.25) × SECONDS + 가감속 (기본값이면 약 60mm)
# Python 버전: hw/py/stepper_speed_sweep.py
python3 - <<'EOF'
import pigpio, time

STEP, DIR, EN = 20, 21, 16
MICRO     = 8               # 1/8 스텝
SPEEDS_HZ = [200, 250]      # 안정 속도: 200Hz = 5mm/s, 250Hz = 6.25mm/s (1/8 기준)
DIRECTION = 1               # 0 또는 1
SECONDS   = 5               # 속도마다 유지 시간(초)
RAMP_S    = 0.3             # 가속·감속 단계 하나당 시간(초)
START_HZ  = 160             # 출발 주파수

SPR  = 200 * MICRO          # 1바퀴 스텝 수
SPMM = SPR / 40             # 1mm = 40스텝 (GT2 20T)

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
for p in (STEP, DIR, EN):
    pi.set_mode(p, pigpio.OUTPUT)

def pwm(hz):
    real = pi.set_PWM_frequency(STEP, hz)
    pi.set_PWM_dutycycle(STEP, 128)
    return real

try:
    pi.write(STEP, 0)
    pi.write(DIR, DIRECTION)
    pi.write(EN, 0)
    time.sleep(0.05)
    pwm(START_HZ)                                   # 천천히 출발
    time.sleep(RAMP_S)
    for hz in SPEEDS_HZ:
        real = pwm(hz)
        print(f"1/{MICRO} · {real / SPMM:.2f}mm/s · {real}Hz · 초당 {real / SPR:.3f}바퀴")
        time.sleep(SECONDS)
    for hz in reversed([START_HZ] + SPEEDS_HZ[:-1]):  # 거꾸로 감속
        pwm(hz)
        time.sleep(RAMP_S)
    print("감속 후 정지")
except KeyboardInterrupt:
    print("중단됨")
finally:
    pi.set_PWM_dutycycle(STEP, 0)
    pi.write(STEP, 0)
    pi.write(EN, 1)
    pi.stop()
EOF
