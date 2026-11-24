# limit_test.py
import RPi.GPIO as GPIO
import time

PINS = {"모터쪽(BCM5)": 5, "반대쪽(BCM6)": 6}

GPIO.setmode(GPIO.BCM)
for p in PINS.values():
    GPIO.setup(p, GPIO.IN)   # 풀업/풀다운 없이 읽기 (모듈에 풀업이 있는지 확인용)

try:
    prev = None
    while True:
        cur = tuple(GPIO.input(p) for p in PINS.values())
        if cur != prev:
            print(time.strftime("%H:%M:%S"),
                  " | ".join(f"{n}={v}" for n, v in zip(PINS, cur)))
            prev = cur
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()