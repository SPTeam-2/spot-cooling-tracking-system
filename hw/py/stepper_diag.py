# 모터가 소리만 나고 안 돌 때 원인 가르기
import pigpio, time, sys

STEP, DIR, EN = 20, 21, 16

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpiod 미실행. sudo pigpiod 먼저")
for p in (STEP, DIR, EN):
    pi.set_mode(p, pigpio.OUTPUT)
pi.set_PWM_dutycycle(STEP, 0)

def run(freq, secs):
    actual = pi.set_PWM_frequency(STEP, freq)
    print(f"   {freq}Hz 요청 -> 실제 {actual}Hz, {secs}초")
    pi.set_PWM_dutycycle(STEP, 128)
    time.sleep(secs)
    pi.set_PWM_dutycycle(STEP, 0)

try:
    pi.write(EN, 1)
    input("[1] EN 끔. 풀리를 손으로 돌려보세요 (자유롭게 돌아야 정상). Enter")

    pi.write(EN, 0)
    pi.write(DIR, 1)
    input("[2] EN 켬, 펄스 없음. 다시 돌려보세요 (뻑뻑해야 정상). Enter")

    print("[3] 아주 느리게")
    run(20, 8)
    run(40, 8)
    print("[4] 보통")
    run(160, 8)
finally:
    pi.set_PWM_dutycycle(STEP, 0)
    pi.write(EN, 1)
    pi.stop()
