# 가감속 없이 한 방향 고정 주파수로만 돌리는 최소 시험
import pigpio, time, sys

STEP, DIR, EN = 20, 21, 16
FREQ      = 160    # Hz (1/8스텝에서 4mm/s)
STEPS     = 1600   # 풀리 1바퀴 = 40mm
DIRECTION = 1      # 0 = 모터 쪽, 1 = 모터에서 멀어지는 쪽

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpiod 미실행. sudo pigpiod 먼저")
for p in (STEP, DIR, EN):
    pi.set_mode(p, pigpio.OUTPUT)

pi.write(EN, 0)          # A4988 EN은 LOW가 활성
pi.write(DIR, DIRECTION)
time.sleep(0.1)

count = 0
def on_edge(gpio, level, tick):
    global count
    count += 1
cb = pi.callback(STEP, pigpio.RISING_EDGE, on_edge)

actual = pi.set_PWM_frequency(STEP, FREQ)
print(f"요청 {FREQ}Hz -> 실제 {actual}Hz")
pi.set_PWM_dutycycle(STEP, 128)

limit = STEPS / actual + 3
t0 = time.time()
while count < STEPS and time.time() - t0 < limit:
    time.sleep(0.01)

pi.set_PWM_dutycycle(STEP, 0)
time.sleep(0.05)
print(f"보낸 펄스 {count}/{STEPS}, 걸린 시간 {time.time()-t0:.1f}초")
cb.cancel()
pi.write(EN, 1)
pi.stop()
