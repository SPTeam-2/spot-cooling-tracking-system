#!/bin/bash
# 목적: 팬틸트 서보(pan SG90, tilt MG90S) 동작과 브래킷에 걸리지 않는 안전 범위 확인
# 동작: 범위마다 1500(가운데) → 아래 값 → 위 값 → 1500, 좁은 범위부터 넓혀 감
# 실행: bash hw/bash/servo_range_test.bash   (먼저 pgrep pigpiod || sudo pigpiod, 12V 연결 = 서보 5V)
# 확인: 걸리거나 "지이잉" 소리가 나는 범위의 직전이 안전 최대값 → #12 팬틸트 각도 제한값으로 사용
# 주의: 걸리거나 소리가 나면 바로 Ctrl+C. 대상은 TARGETS로 선택
# Python 버전: hw/py/servo_check.py
python3 - <<'EOF'
import pigpio, time

PAN, TILT = 12, 13                                      # BCM (물리 32번, 33번)
TARGETS   = ["pan", "tilt"]                             # ["pan"], ["tilt"]로 하나만 가능
RANGES    = [(1300, 1700), (1100, 1900), (1000, 2000)]  # 펄스 폭 범위(µs), 500~2500 안에서
HOLD      = 1                                           # 위치마다 유지 시간(초)

PINS = {"pan": PAN, "tilt": TILT}
pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")

try:
    for name in TARGETS:
        pin = PINS[name]
        for lo, hi in RANGES:
            print(f"{name} · 범위 {lo} ~ {hi}")
            for pw in (1500, lo, hi, 1500):
                pi.set_servo_pulsewidth(pin, pw)
                time.sleep(HOLD)
except KeyboardInterrupt:
    print("중단됨")
finally:
    pi.set_servo_pulsewidth(PAN, 0)
    pi.set_servo_pulsewidth(TILT, 0)
    pi.stop()
EOF
