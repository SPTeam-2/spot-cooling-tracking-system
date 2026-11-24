# 팬(BCM18) + 팬틸트 서보(BCM12/13) 동시 동작 확인
# 통합 코드에 넣기 전, 하드웨어가 서로 간섭하는지만 보는 시험
# pigpio 소프트웨어 PWM 사용 -> GPIO 12·18 하드웨어 PWM 채널 충돌 회피
import pigpio, time, sys

FAN, PAN, TILT = 18, 12, 13

FAN_FREQ    = 1000          # Hz (pigpio 허용값)
PAN_CENTER,  PAN_SPAN  = 1550, 250   # us
TILT_CENTER, TILT_SPAN = 1200, 100   # us, 틸트 나사 때문에 좁게
HOLD = 0.6

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpiod 미실행. sudo pigpiod 먼저")

def fan(pct):
    pi.set_PWM_dutycycle(FAN, int(255 * pct / 100))
    print(f"    팬 {pct}%")

def servo(pin, us, name):
    pi.set_servo_pulsewidth(pin, us)
    print(f"    {name} {us}us")

def sweep(pin, center, span, name):
    for us in (center - span, center + span, center):
        servo(pin, us, name)
        time.sleep(HOLD)

def all_off():
    pi.set_PWM_dutycycle(FAN, 0)
    pi.set_servo_pulsewidth(PAN, 0)
    pi.set_servo_pulsewidth(TILT, 0)

try:
    pi.set_mode(FAN, pigpio.OUTPUT)
    actual = pi.set_PWM_frequency(FAN, FAN_FREQ)
    print(f"팬 PWM 요청 {FAN_FREQ}Hz -> 실제 {actual}Hz\n")

    print("[1] 서보만 (팬 정지)")
    servo(PAN, PAN_CENTER, "pan"); servo(TILT, TILT_CENTER, "tilt")
    time.sleep(1)
    sweep(PAN, PAN_CENTER, PAN_SPAN, "pan")
    sweep(TILT, TILT_CENTER, TILT_SPAN, "tilt")

    print("\n[2] 팬만 (서보 정지)")
    pi.set_servo_pulsewidth(PAN, 0); pi.set_servo_pulsewidth(TILT, 0)
    time.sleep(0.5)
    for pct in (30, 60, 100):
        fan(pct); time.sleep(1.5)
    fan(0); time.sleep(1)

    print("\n[3] 동시 - 팬 100% + 서보 움직임")
    fan(100); time.sleep(1)
    servo(PAN, PAN_CENTER, "pan"); servo(TILT, TILT_CENTER, "tilt")
    time.sleep(1)
    for _ in range(2):
        sweep(PAN, PAN_CENTER, PAN_SPAN, "pan")
        sweep(TILT, TILT_CENTER, TILT_SPAN, "tilt")

    print("\n[4] 동시 - 서보 움직이는 중 팬 듀티 변경")
    for pct in (0, 100, 30, 100, 0):
        fan(pct)
        servo(PAN, PAN_CENTER + PAN_SPAN, "pan"); time.sleep(HOLD)
        servo(PAN, PAN_CENTER - PAN_SPAN, "pan"); time.sleep(HOLD)
    servo(PAN, PAN_CENTER, "pan")
    time.sleep(1)

    print("\n완료")
except KeyboardInterrupt:
    print("\n중단")
finally:
    all_off()
    time.sleep(0.2)
    pi.stop()
