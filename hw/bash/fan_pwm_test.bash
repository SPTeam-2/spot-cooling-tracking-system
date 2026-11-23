#!/bin/bash
# 목적: D4184 MOSFET 모듈로 팬 세기가 PWM duty에 따라 바뀌는지 확인
# 동작: 꺼짐 → 100% → 50% → 25% → 꺼짐, 단계마다 HOLD초 유지
# 실행: bash hw/bash/fan_pwm_test.bash   (먼저 pgrep pigpiod || sudo pigpiod, 12V 연결)
# 배선: 팬 빨강 → D4184 플러스, 팬 검정 → LOAD, 12V → 플러스·마이너스, PWM ← 브레드보드 h27 (Pi 12번 = BCM 18)
# 주의: D4184 신호 쪽 옵토커플러 때문에 1000Hz 정도의 낮은 주파수 사용
#       12V를 넣자마자 팬이 최대로 돌면 팬 검정이 LOAD에 있는지 확인
# Python 버전: hw/py/fan_pwm_check.py
python3 - <<'EOF'
import pigpio, time

FAN  = 18                 # BCM 18 (물리 12번)
FREQ = 1000               # PWM 주파수(Hz)
HOLD = 3                  # 단계마다 유지 시간(초)

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
pi.set_mode(FAN, pigpio.OUTPUT)

try:
    real = pi.set_PWM_frequency(FAN, FREQ)
    print(f"PWM {real}Hz")
    for duty, label in ((0, "꺼짐"), (255, "100%"), (128, "50%"), (64, "25%"), (0, "꺼짐")):
        print(label)
        pi.set_PWM_dutycycle(FAN, duty)
        time.sleep(HOLD)
except KeyboardInterrupt:
    print("중단됨")
finally:
    pi.set_PWM_dutycycle(FAN, 0)
    pi.stop()
EOF
