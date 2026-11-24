"""pi_server용 초음파(HC-SR04) 거리 측정. 별도 스레드에서 계속 재고, 최근 값의 중앙값을 돌려준다.

측정 방식은 구동부 점검 스크립트(hw/py/ultrasonic_test.py)와 같음: 10µs Trig → Echo 에지 콜백으로 시간 측정.
"""

from __future__ import annotations

import statistics
import threading
import time
from collections import deque

MM_PER_US = 0.343        # 상온 음속 343m/s
GAP_S = 0.06             # 측정 간격, 반향 잔류 방지
ECHO_TIMEOUT_S = 0.04
VALID_MM = (20, 4000)
WINDOW = 5               # 중앙값에 쓰는 최근 측정 수
FRESH_S = 0.5            # 이보다 오래된 측정은 버림


class Sonar:
    def __init__(self, measure, *, clock=time.monotonic, close=None):
        """measure(): 한 번 재서 mm 또는 None."""
        self._measure, self._clock, self._close = measure, clock, close
        self._vals = deque(maxlen=WINDOW)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def sample(self):
        d = self._measure()
        if d is not None and VALID_MM[0] <= d <= VALID_MM[1]:
            with self._lock:
                self._vals.append((self._clock(), d))

    def read_mm(self):
        now = self._clock()
        with self._lock:
            vals = [d for t, d in self._vals if now - t <= FRESH_S]
        return statistics.median(vals) if vals else None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            try:
                self.sample()
            except Exception as e:
                print(f"[파이] 초음파 측정 실패: {e}", flush=True)
                return
            self._stop.wait(GAP_S)

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=0.5)
        if self._close:
            self._close()


class SimSonar:
    """--dummy: 고정 거리(없으면 None)."""

    def __init__(self, mm=None):
        self.mm = mm

    def read_mm(self):
        return self.mm

    def close(self):
        pass


def build_sonar(pins):
    """실장비용 Sonar. pigpiod가 떠 있어야 한다."""
    import pigpio

    trig, echo = pins["ultrasonic"]["trig"], pins["ultrasonic"]["echo"]
    pi = pigpio.pi("127.0.0.1", 8888)
    if not pi.connected:
        raise RuntimeError("pigpiod 미실행 (sudo pigpiod)")
    pi.set_mode(trig, pigpio.OUTPUT)
    pi.set_mode(echo, pigpio.INPUT)
    pi.write(trig, 0)
    edge = {"rise": None, "dur": None}
    done = threading.Event()

    def on_edge(_gpio, level, tick):
        if level == 1:
            edge["rise"] = tick
        elif level == 0 and edge["rise"] is not None:
            edge["dur"] = pigpio.tickDiff(edge["rise"], tick)
            edge["rise"] = None
            done.set()

    cb = pi.callback(echo, pigpio.EITHER_EDGE, on_edge)

    def measure():
        edge["rise"] = edge["dur"] = None
        done.clear()
        pi.gpio_trigger(trig, 10, 1)
        if not done.wait(ECHO_TIMEOUT_S):
            return None
        return edge["dur"] * MM_PER_US / 2

    def close():
        cb.cancel()
        pi.write(trig, 0)
        pi.stop()

    return Sonar(measure, close=close).start()
