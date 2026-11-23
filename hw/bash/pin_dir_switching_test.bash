#!/bin/bash
# 목적: Pi DIR 출력 핀(BCM 21, 물리 40번)이 0/1을 제대로 내보내는지 확인 (멀티미터 없이)
# 준비: 브레드보드로 가는 DIR 선을 Pi에서 빼고, 점퍼로 물리 40번 ↔ 물리 37번(BCM 26)을 직접 연결
# 실행: bash hw/bash/pin_dir_switching_test.bash   (먼저 pgrep pigpiod || sudo pigpiod)
# 결과: 읽은 값이 0, 1, 0, 1로 따라오면 Pi 40번 출력 정상 → 문제는 Pi와 브레드보드 사이 선
# 주의: 끝나면 40↔37 점퍼를 빼고 DIR 선을 다시 Pi 40번에 연결
python3 - <<'EOF'
import pigpio, time
DIR, TEST = 21, 26       # 물리 40번(DIR), 물리 37번(읽기용)
pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
pi.set_mode(DIR, pigpio.OUTPUT)
pi.set_mode(TEST, pigpio.INPUT)
pi.set_pull_up_down(TEST, pigpio.PUD_OFF)
for v in (0, 1, 0, 1):
    pi.write(DIR, v)
    time.sleep(0.1)
    print(f"DIR에 {v} 출력 → 37번에서 읽은 값: {pi.read(TEST)}")
pi.write(DIR, 0)
pi.stop()
EOF