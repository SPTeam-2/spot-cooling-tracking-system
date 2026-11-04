"""라즈베리파이 구동 서버 — 노트북 명령(UDP)을 받아 캐리지 이동 + 팬 제어.

실행: python3 src/comms/pi_server.py
GPIO가 없으면 더미 모드로 동작만 출력하므로 PC에서도 테스트된다.
실제 모터·팬 GPIO는 # TODO 부분을 하드웨어 배선 후 채운다.
"""

from __future__ import annotations

import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from protocol import COMMS_TIMEOUT_S, PORT, Command

_CFG_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "config")
try:
    import yaml
    _cfg = yaml.safe_load(open(os.path.join(_CFG_DIR, "coordinate.yaml")))
except Exception:
    _cfg = {}

try:
    import RPi.GPIO as GPIO  # noqa: F401
    ON_PI = True
except Exception:
    ON_PI = False


class Actuators:
    def __init__(self):
        rail = _cfg.get("rail", {})
        st = _cfg.get("stepper", {})
        self.travel_mm = rail.get("travel_mm", 530)
        self.mm_per_step = (st.get("belt_pitch_mm", 2) * st.get("pulley_teeth", 20)) \
            / (st.get("steps_per_rev", 200) * st.get("microstep", 16))
        self.deadzone_mm = _cfg.get("control", {}).get("deadzone_mm", 20)
        self.max_speed_mm_s = 300.0
        self.current_mm = 0.0
        self.duty = 0
        self._fan = None
        if ON_PI:
            self._init_gpio()

    def _init_gpio(self):
        from fan.fan_control import FanController
        pins = yaml.safe_load(open(os.path.join(_CFG_DIR, "pins.yaml")))
        self._fan = FanController(pins["fan"]["pwm"])
        # TODO: 스테퍼 STEP/DIR/EN 핀 초기화

    def set_fan(self, duty: int):
        self.duty = duty
        if self._fan:
            self._fan.set_duty(duty)

    def step_toward(self, target_mm: float, dt: float):
        target_mm = max(0.0, min(self.travel_mm, target_mm))
        diff = target_mm - self.current_mm
        if abs(diff) <= self.deadzone_mm:
            return
        move = max(-self.max_speed_mm_s * dt, min(self.max_speed_mm_s * dt, diff))
        n_steps = int(abs(move) / self.mm_per_step)
        self.current_mm += (1 if move > 0 else -1) * n_steps * self.mm_per_step
        if ON_PI:
            pass  # TODO: DIR 세팅 후 STEP 핀 n_steps번 펄스

    def stop(self):
        self.set_fan(0)


_state = {"cmd": Command(), "last_rx": 0.0}
_lock = threading.Lock()


def udp_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", PORT))
    print(f"[파이] UDP {PORT} 수신 대기 ({'실장비' if ON_PI else '더미'})")
    while True:
        data, _ = sock.recvfrom(1024)
        try:
            cmd = Command.from_bytes(data)
        except Exception:
            continue
        with _lock:
            _state["cmd"] = cmd
            _state["last_rx"] = time.time()


def control_loop(act: Actuators, hz: int = 100):
    period = 1.0 / hz
    last_log = 0.0
    while True:
        with _lock:
            cmd = _state["cmd"]
            last_rx = _state["last_rx"]
        linked = bool(last_rx) and (time.time() - last_rx) < COMMS_TIMEOUT_S

        if linked and cmd.active:
            act.step_toward(cmd.target_mm, period)
            act.set_fan(cmd.fan_duty)
        else:
            act.stop()   # 끊김 또는 대상 없음

        now = time.time()
        if now - last_log > 0.5:
            tag = "연결" if linked else "끊김/대기"
            print(f"  [{tag}] 캐리지 {act.current_mm:6.1f}mm → {cmd.target_mm:6.1f}mm | 팬 {act.duty:>3}%")
            last_log = now
        time.sleep(period)


def main():
    act = Actuators()
    threading.Thread(target=udp_receiver, daemon=True).start()
    try:
        control_loop(act)
    except KeyboardInterrupt:
        act.stop()


if __name__ == "__main__":
    main()
