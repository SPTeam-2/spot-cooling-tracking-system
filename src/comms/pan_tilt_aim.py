"""pi_server용 팬틸트 제어: 노트북이 보낸 pan_deg·tilt_deg를 구동부 MoveToAngle로 따라감.

시작 시 중립(0, 0) → 받은 각도를 허용 범위로 잘라 이동. 각도 값은 config/pan_tilt_test.yaml에서 읽고,
비어 있으면 서버는 팬틸트 없이 레일·팬만 동작한다.
"""

from __future__ import annotations

import os

import yaml

from pan_tilt.function_pan_tilt import AXES, AxisConfig, PanTiltFunctions
from pan_tilt.move_to_angle import AngleStatus, MoveToAngle, MoveToAngleConfig

DEADBAND_DEG = 1.0   # 이보다 작은 각도 변화는 무시 (깊이 흔들림으로 서보가 떨지 않게)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PanTiltAim:
    def __init__(self, pan_tilt, mover, *, on_close=()):
        self.pan_tilt, self.mover = pan_tilt, mover
        self._on_close = on_close
        self.fault = None
        self._goal = None
        pan_tilt.center()

    @property
    def angles(self):
        return self.pan_tilt.angle("pan"), self.pan_tilt.angle("tilt")

    def tick(self, pan_deg, tilt_deg):
        """pan_deg=None이면 그 자리에서 멈춤(대상 없음·통신 끊김)."""
        if self.fault:
            return
        try:
            if pan_deg is None:
                self._goal = None
                if self.mover.result.status == AngleStatus.MOVING:
                    self.mover.stop()
                return
            cfg = self.pan_tilt.config
            goal = (_clamp(float(pan_deg), cfg["pan"].min_deg, cfg["pan"].max_deg),
                    _clamp(float(tilt_deg), cfg["tilt"].min_deg, cfg["tilt"].max_deg))
            if self._goal is None or any(abs(g - p) >= DEADBAND_DEG for g, p in zip(goal, self._goal)):
                self.mover.set_target(*goal)
                self._goal = goal
            r = self.mover.update()
            if r.status == AngleStatus.FAILED:
                self.fault = f"팬틸트 오류: {r.error}"
        except Exception as e:
            self.fault = f"팬틸트 오류: {e}"

    def close(self):
        try:
            self.mover.stop()
            self.pan_tilt.off()
        except Exception as e:
            print(f"[파이] 팬틸트 끄기 실패: {e}")
        for action in self._on_close:
            try:
                action()
            except Exception as e:
                print(f"[파이] 팬틸트 정리 실패: {e}")


class SimPanTilt:
    """--dummy: 받은 각도로 바로 바뀌는 가상 팬틸트."""

    def __init__(self):
        self.fault = None
        self.angles = (0.0, 0.0)

    def tick(self, pan_deg, tilt_deg):
        if pan_deg is not None:
            self.angles = (float(pan_deg), float(tilt_deg))

    def close(self):
        pass


def load_pan_tilt_settings(cfg_dir):
    """pan_tilt_test.yaml을 읽는다. 비어 있는 값이 있으면 (None, 이유)를 돌려준다."""
    with open(os.path.join(cfg_dir, "pan_tilt_test.yaml"), encoding="utf-8") as f:
        v = yaml.safe_load(f) or {}
    keys = ("center_pulse_us", "us_per_deg", "min_deg", "max_deg")
    missing = [f"{a}.{k}" for a in AXES for k in keys if (v.get(a) or {}).get(k) is None]
    if v.get("max_speed_deg_s") is None:
        missing.append("max_speed_deg_s")
    if missing:
        return None, "pan_tilt_test.yaml 값이 비어 있음: " + ", ".join(missing)
    return {
        "axes": {a: AxisConfig(**{k: v[a][k] for k in keys}) for a in AXES},
        "move": MoveToAngleConfig(max_speed_deg_s=v["max_speed_deg_s"]),
        "io_timeout": v.get("io_timeout") or 0.25,
    }, None


def build_pan_tilt(pins, settings):
    """실장비용 PanTiltAim 생성. 실패하면 이미 연 장치는 정리하고 예외를 그대로 던진다."""
    from pan_tilt.hw_servo import ServoHardware
    from pan_tilt.pigpio_servo_connection import PigpioServoConnection

    servo_pins = pins["servo"]
    reserved = tuple(p for name, sec in pins.items() if name != "servo" and isinstance(sec, dict)
                     for p in sec.values() if type(p) is int)
    conn = PigpioServoConnection(timeout_s=settings["io_timeout"])
    servos = {}
    try:
        for a in AXES:
            servos[a] = ServoHardware(servo_pins[a], reserved_pins=reserved, connection=conn)
        pan_tilt = PanTiltFunctions(servos["pan"], servos["tilt"],
                                    settings["axes"]["pan"], settings["axes"]["tilt"])
        mover = MoveToAngle(pan_tilt, settings["move"])
        return PanTiltAim(pan_tilt, mover,
                          on_close=tuple(s.cleanup for s in servos.values()) + (conn.stop,))
    except Exception:
        for s in servos.values():
            try:
                s.cleanup()
            except Exception:
                pass
        conn.stop()
        raise
