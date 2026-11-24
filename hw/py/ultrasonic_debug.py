# 초음파 무응답 원인 가르기
import pigpio, time, sys

TRIG, ECHO = 23, 24

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpiod 미실행. sudo pigpiod 먼저")
pi.set_mode(TRIG, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)
pi.write(TRIG, 0)
time.sleep(0.1)

print(f"Trig 쏘기 전 Echo 레벨: {pi.read(ECHO)}   (0이 정상)\n")

n = [0]
def on_edge(g, level, tick):
    n[0] += 1
cb = pi.callback(ECHO, pigpio.EITHER_EDGE, on_edge)

for i in range(10):
    before = n[0]
    pi.gpio_trigger(TRIG, 10, 1)
    time.sleep(0.08)
    print(f"  {i+1:2}회: 에지 {n[0]-before}개, 현재 Echo {pi.read(ECHO)}")
cb.cancel()

print("\nTrig 핀을 3초간 HIGH로 둡니다 (멀티미터 있으면 3.3V 확인)")
pi.write(TRIG, 1); time.sleep(3); pi.write(TRIG, 0)
pi.stop()
