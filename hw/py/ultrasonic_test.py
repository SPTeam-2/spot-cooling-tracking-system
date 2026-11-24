# HC-SR04 초음파 센서 동작 확인
# 배선: Trig BCM23(물리 16), Echo BCM24(물리 18, 1k/2k 분압 거쳐서), VCC 5V, GND
# 실행: pgrep pigpiod || sudo pigpiod  ->  python3 hw/py/ultrasonic_test.py
import pigpio, time, statistics, sys

TRIG, ECHO = 23, 24
SAMPLES   = 5       # 한 줄 출력에 쓰는 측정 횟수 (중앙값)
GAP_S     = 0.06    # 측정 사이 간격, 반향 잔류 방지로 60ms 이상
PERIOD_S  = 0.5     # 화면 갱신 주기
TIMEOUT_S = 0.04    # 에코 대기 상한 (약 6.8m)
MM_PER_US = 0.343   # 상온 음속 343m/s
MIN_MM, MAX_MM = 20, 4000

pi = pigpio.pi()
if not pi.connected:
    sys.exit("pigpiod 미실행. sudo pigpiod 먼저")

pi.set_mode(TRIG, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)
pi.write(TRIG, 0)
time.sleep(0.05)

_rise = _dur = None

def on_edge(gpio, level, tick):
    global _rise, _dur
    if level == 1:
        _rise = tick
    elif level == 0 and _rise is not None:
        _dur = pigpio.tickDiff(_rise, tick)
        _rise = None

cb = pi.callback(ECHO, pigpio.EITHER_EDGE, on_edge)

def measure():
    """한 번 측정 -> 거리(mm) 또는 None(무응답)"""
    global _rise, _dur
    _rise = _dur = None
    pi.gpio_trigger(TRIG, 10, 1)          # 10us HIGH 펄스
    t0 = time.time()
    while _dur is None and time.time() - t0 < TIMEOUT_S:
        time.sleep(0.0005)
    return None if _dur is None else _dur * MM_PER_US / 2

try:
    print(f"Trig BCM{TRIG} / Echo BCM{ECHO}  -  Ctrl+C 종료\n")
    while True:
        vals = []
        for _ in range(SAMPLES):
            d = measure()
            if d is not None:
                vals.append(d)
            time.sleep(GAP_S)
        if not vals:
            print("  응답 없음 - 배선/분압 확인")
        else:
            raw = " ".join(f"{v:.0f}" for v in vals)
            ok = [v for v in vals if MIN_MM <= v <= MAX_MM]
            if ok:
                print(f"  {statistics.median(ok):6.0f}mm  (유효 {len(ok)}/{SAMPLES})  [{raw}]")
            else:
                print(f"  범위 밖  [{raw}]")
        time.sleep(PERIOD_S)
except KeyboardInterrupt:
    print("\n종료")
finally:
    cb.cancel()
    pi.write(TRIG, 0)
    pi.stop()
