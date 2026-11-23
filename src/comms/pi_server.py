"""라즈베리파이 구동 서버 — 노트북 명령(UDP)을 받아 캐리지 이동 + 팬 제어.

실행: python3 src/comms/pi_server.py   (하드웨어 없이 확인: --dummy)
실장비: 시작하면 먼저 Homing(원점 찾기) → 끝나면 노트북이 보낸 target_mm를 따라감.
속도·Homing 값은 구동부 현장 시험 파일(config/*_test.yaml)을 그대로 읽는다.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from protocol import COMMS_TIMEOUT_S, PORT, Command
from comms.carriage_rail import SimRail, build_carriage_rail, load_rail_settings

_CFG_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "config")
LOOP_S = 0.001     # 구동부 update가 한 번에 최대 한 STEP이라 촘촘히 돌림


class Fan:
    def __init__(self, on_pi: bool, pins: dict | None = None):
        self.duty = 0
        self._ctl = None
        if on_pi:
            from fan.fan_control import FanController
            self._ctl = FanController(pins["fan"]["pwm"])

    def set(self, duty: int):
        if duty == self.duty:
            return
        self.duty = duty
        if self._ctl:
            self._ctl.set_duty(duty)

    def close(self):
        self.set(0)
        if self._ctl:
            self._ctl.cleanup()


_state = {"cmd": Command(), "last_rx": 0.0}
_lock = threading.Lock()


def udp_receiver(sock: socket.socket):
    while True:
        data, _ = sock.recvfrom(1024)
        try:
            cmd = Command.from_bytes(data)
        except Exception:
            continue
        with _lock:
            _state["cmd"] = cmd
            _state["last_rx"] = time.time()


def control_step(rail, fan, cmd: Command, linked: bool):
    run = linked and cmd.active and not rail.fault
    rail.tick(cmd.target_mm if run else None)
    fan.set(cmd.fan_duty if run and not rail.fault else 0)


def control_loop(rail, fan):
    last_log = 0.0
    while True:
        with _lock:
            cmd = _state["cmd"]
            last_rx = _state["last_rx"]
        linked = bool(last_rx) and (time.time() - last_rx) < COMMS_TIMEOUT_S
        control_step(rail, fan, cmd, linked)

        now = time.time()
        if now - last_log > 0.5:
            tag = f"오류: {rail.fault}" if rail.fault else rail.state if rail.state != "준비" \
                else "연결" if linked else "끊김/대기"
            pos = rail.position_mm
            pos_s = "  ?  " if pos is None else f"{pos:6.1f}"
            print(f"  [{tag}] 캐리지 {pos_s}mm → {cmd.target_mm:6.1f}mm | 팬 {fan.duty:>3}%", flush=True)
            last_log = now
        time.sleep(LOOP_S)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dummy", action="store_true", help="GPIO 없이 가상 레일로 동작만 출력")
    dummy = ap.parse_args().dummy

    with open(os.path.join(_CFG_DIR, "coordinate.yaml"), encoding="utf-8") as f:
        coord = yaml.safe_load(f)
    fan = rail = None
    try:
        if dummy:
            rail = SimRail(travel_mm=coord["rail"]["travel_mm"],
                           deadzone_mm=coord.get("control", {}).get("deadzone_mm", 20))
            fan = Fan(on_pi=False)
        else:
            with open(os.path.join(_CFG_DIR, "pins.yaml"), encoding="utf-8") as f:
                pins = yaml.safe_load(f)
            settings = load_rail_settings(_CFG_DIR)
            fan = Fan(on_pi=True, pins=pins)
            rail = build_carriage_rail(pins, settings)
    except Exception as e:
        if fan:
            fan.close()
        sys.exit(f"[파이] 초기화 실패: {e}\n  하드웨어 없이 확인하려면 --dummy")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", PORT))
    except OSError as e:
        rail.close()
        fan.close()
        sys.exit(f"[파이] UDP {PORT} 포트를 열 수 없음: {e}\n  다른 pi_server가 이미 실행 중인지 확인")
    threading.Thread(target=udp_receiver, args=(sock,), daemon=True).start()
    print(f"[파이] UDP {PORT} 수신 대기 ({'더미' if dummy else '실장비'}) / Ctrl+C: 종료", flush=True)
    try:
        control_loop(rail, fan)
    except KeyboardInterrupt:
        pass
    finally:
        rail.close()
        fan.close()


if __name__ == "__main__":
    main()
