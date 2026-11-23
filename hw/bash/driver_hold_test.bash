#!/bin/bash
# 목적: A4988이 두 코일 모두에 전류를 보내는지 손으로 확인 (홀딩 토크)
# 동작: STEP 펄스 없이 EN만 켜고 SECONDS초 유지 → EN 끄기
# 실행: bash hw/bash/driver_hold_test.bash   (먼저 pgrep pigpiod || sudo pigpiod, 12V 연결)
# 확인: 유지하는 동안 축을 손으로 천천히 돌려 보기
#       모든 위치에서 고르게 버팀 → 정상 / 일부 위치에서만 걸리거나 거의 안 버팀 → A4988 출력·코일 확인
#       끝난 뒤 손으로 쉽게 돌아가야 EN 정상 (계속 버티면 EN 선이 안 들어가는 것)
# Python 버전: hw/py/stepper_hold_check.py
python3 - <<'EOF'
import pigpio, time

STEP, EN = 20, 16
SECONDS  = 10             # EN 유지 시간(초)

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
for p in (STEP, EN):
    pi.set_mode(p, pigpio.OUTPUT)

try:
    pi.write(STEP, 0)
    pi.write(EN, 0)
    print(f"EN 켜짐: {SECONDS}초 동안 축을 손으로 돌려 보세요")
    time.sleep(SECONDS)
except KeyboardInterrupt:
    print("중단됨")
finally:
    pi.write(EN, 1)
    print("EN 꺼짐 (이제 손으로 쉽게 돌아가야 정상)")
    pi.stop()
EOF
