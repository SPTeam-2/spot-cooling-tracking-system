# 코일이 실제로 드라이버에 걸려 있는지 유지 토크로 확인
# 선을 뺐다 꽂을 때는 반드시 12V를 끄고 할 것
import pigpio, time, sys

STEP, DIR, EN = 20, 21, 16
HOLD_S = 20

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpiod 미실행. sudo pigpiod 먼저")
for p in (STEP, DIR, EN):
    pi.set_mode(p, pigpio.OUTPUT)
pi.set_PWM_dutycycle(STEP, 0)
pi.write(STEP, 0)
pi.write(DIR, 1)

try:
    pi.write(EN, 0)
    print(f"{HOLD_S}초간 코일에 전류를 흘립니다. 축을 손으로 돌려 느낌을 보세요")
    time.sleep(HOLD_S)
finally:
    pi.write(EN, 1)
    pi.stop()
    print("끝 (EN 해제)")
