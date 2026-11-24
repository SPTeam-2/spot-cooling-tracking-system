#!/bin/bash
# 테스트 완료, 캐리지 이동 후 좌표에 맞출 예정이라 이거는 더이상 안 봐도 됨.
# 목적: 보낸 스텝만큼 정확히 도는지 여러 번 반복해서 확인 (1/8 스텝)
# 동작: REVS바퀴를 RUNS번 반복, 회차마다 PAUSE초 멈춤 (스텝 수는 정확, 속도는 Python sleep이라 약간 흔들림)
# 실행: bash hw/bash/carriage_rev_test.bash   (먼저 pgrep pigpiod || sudo pigpiod)
# 확인: 축이나 풀리에 네임펜으로 표시 → 회차마다 표시가 제자리에 오는지
#       회전량이 매번 다르면 MS 핀 접촉 불량 의심 (예: 1/8에서 2바퀴 → MS1이 끊겨 1/4로 동작)
# Python 버전: hw/py/stepper_rev_check.py
python3 - <<'EOF'
import pigpio, time

STEP, DIR, EN = 20, 21, 16
MICRO       = 8           # 1/8 스텝 (MS1·MS2 = 3.3V, MS3 미연결)
REVS        = 1           # 한 번에 돌릴 바퀴 수
RUNS        = 5           # 반복 횟수
SEC_PER_REV = 4           # 1바퀴에 걸릴 시간(초), 클수록 느림
DIRECTION   = 0           # 0 또는 1
PAUSE       = 3           # 회차 사이 확인 시간(초)

STEPS = int(round(200 * MICRO * REVS))
DELAY = SEC_PER_REV / (200 * MICRO) / 2

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
for p in (STEP, DIR, EN):
    pi.set_mode(p, pigpio.OUTPUT)

try:
    pi.write(STEP, 0)
    pi.write(DIR, DIRECTION)
    pi.write(EN, 0)
    time.sleep(0.5)
    print(f"1/{MICRO} · {STEPS}스텝 = {REVS}바퀴 · 축 표시 위치 확인")
    for r in range(1, RUNS + 1):
        t = time.time()
        for _ in range(STEPS):
            pi.write(STEP, 1); time.sleep(DELAY)
            pi.write(STEP, 0); time.sleep(DELAY)
        print(f"  {r}회차 완료 ({time.time() - t:.1f}초): 표시가 제자리인지 확인")
        time.sleep(PAUSE)
except KeyboardInterrupt:
    print("중단됨")
finally:
    pi.write(STEP, 0)
    pi.write(EN, 1)
    pi.stop()
EOF
