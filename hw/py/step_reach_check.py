# A4988 STEP 핀까지 펄스가 실제로 도착하는지 확인
# 임시 점퍼: A4988 STEP 줄(11번 줄) -> Pi 물리 11번(BCM17)
import pigpio, time, sys

STEP, MON = 20, 17

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpiod 미실행. sudo pigpiod 먼저")
pi.set_mode(STEP, pigpio.OUTPUT)
pi.set_mode(MON, pigpio.INPUT)
pi.set_pull_up_down(MON, pigpio.PUD_DOWN)

n = [0]
cb = pi.callback(MON, pigpio.RISING_EDGE, lambda g, l, t: n.__setitem__(0, n[0] + 1))

actual = pi.set_PWM_frequency(STEP, 100)
pi.set_PWM_dutycycle(STEP, 128)
time.sleep(2)
pi.set_PWM_dutycycle(STEP, 0)
time.sleep(0.1)

print(f"{actual}Hz로 2초 -> 감지된 펄스 {n[0]}개  (약 200개면 정상)")
cb.cancel()
pi.set_pull_up_down(MON, pigpio.PUD_OFF)
pi.stop()
