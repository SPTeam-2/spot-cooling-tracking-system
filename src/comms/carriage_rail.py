"""pi_server용 레일 제어: 시작 시 Homing → 노트북 목표(mm)를 TargetMover로 따라감.

CarriageRail은 구동부 코드(Homing·TargetMover)를 감싸기만 하고, SimRail은 --dummy용 가상 레일.
tick()은 1ms 간격으로 계속 불러야 한다 (구동부 update가 한 번에 최대 한 STEP).
"""

from __future__ import annotations

import os
import time

import yaml

from carriage.drive_geometry import DriveGeometry
from carriage.function_carriage import CarriageFunctions
from carriage.function_limit_switch import LimitSwitchFunctions
from carriage.function_stepper import StepperFunctions
from carriage.homing import HomingConfig, HomingProcedure, HomingStatus
from carriage.target_move import TargetMoveConfig, TargetMover, TargetStatus

HOMING, READY = "원점 찾는 중", "준비"


class CarriageRail:
    def __init__(self, carriage, homing, mover, *, travel_mm, deadzone_mm, on_close=()):
        self.carriage, self.homing, self.mover = carriage, homing, mover
        self.travel_mm, self.deadzone_mm = travel_mm, deadzone_mm
        self._on_close = on_close
        self.state = HOMING
        self.fault = None
        r = homing.start()
        if r.status == HomingStatus.FAILED:
            self.fault = f"Homing 시작 실패: {r.error}"

    @property
    def position_mm(self):
        return self.carriage.position_mm

    def tick(self, target_mm):
        """target_mm=None이면 이동 중단(대상 없음·통신 끊김)."""
        if self.fault:
            return
        try:
            if self.state == HOMING:
                self._tick_homing()
            else:
                self._tick_target(target_mm)
        except Exception as e:
            self._set_fault(f"구동 오류: {e}")

    def _tick_homing(self):
        r = self.homing.update()
        if r.status == HomingStatus.DONE:
            self.state = READY
        elif r.status != HomingStatus.RUNNING:
            self._set_fault(f"Homing {r.status.value}: {r.error}")

    def _tick_target(self, target_mm):
        res = self.mover.result
        moving = res.status == TargetStatus.MOVING
        if target_mm is None:
            if moving:
                self.mover.stop()
            return
        t = max(0.0, min(self.travel_mm, float(target_mm)))
        if self._should_retarget(t, res, moving):
            res = self.mover.set_target(t)
        if self.mover.result.status == TargetStatus.MOVING:
            res = self.mover.update()
        if res.status == TargetStatus.FAILED:
            self._set_fault(f"이동 실패: {res.error}")

    def _should_retarget(self, t, res, moving):
        """새 목표마다 이동을 다시 시작하면 매번 160Hz부터 가속해 거의 못 간다.
        같은 방향으로 더 멀어진 목표는 지금 이동을 끝낸 뒤 이어서 가고,
        방향이 바뀌거나 더 가까워졌을 때만 바로 바꾼다."""
        pos = self.position_mm
        if pos is None:
            return True
        if not moving:
            return abs(t - pos) > self.deadzone_mm
        goal = res.target_mm
        if abs(t - goal) <= self.deadzone_mm:
            return False
        farther_same_way = (goal - pos) * (t - pos) > 0 and abs(t - pos) > abs(goal - pos)
        return not farther_same_way

    def _set_fault(self, msg):
        self.fault = msg
        self.stop()

    def stop(self):
        try:
            if self.homing.result.status == HomingStatus.RUNNING:
                self.homing.update(stop_requested=True)
            if self.mover.result.status == TargetStatus.MOVING:
                self.mover.stop()
        except Exception:
            pass

    def close(self):
        self.stop()
        for action in self._on_close:
            try:
                action()
            except Exception as e:
                print(f"[파이] 정리 실패: {e}")


class SimRail:
    """--dummy: 일정 속도로 목표를 따라가는 가상 레일."""

    def __init__(self, *, travel_mm, deadzone_mm, speed_mm_s=300.0, clock=time.monotonic):
        self.travel_mm, self.deadzone_mm, self.speed = travel_mm, deadzone_mm, speed_mm_s
        self._clock = clock
        self._t = clock()
        self._pos = 0.0
        self._goal = None
        self.state, self.fault = READY, None

    @property
    def position_mm(self):
        return self._pos

    def tick(self, target_mm):
        now = self._clock()
        dt, self._t = now - self._t, now
        if target_mm is None:
            self._goal = None
            return
        t = max(0.0, min(self.travel_mm, float(target_mm)))
        ref = self._pos if self._goal is None else self._goal
        if abs(t - ref) > self.deadzone_mm:
            self._goal = t
        if self._goal is not None:
            diff = self._goal - self._pos
            self._pos += max(-self.speed * dt, min(self.speed * dt, diff))
            if self._pos == self._goal:
                self._goal = None

    def stop(self):
        pass

    def close(self):
        pass


def _load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_rail_settings(cfg_dir):
    """구동부 현장 시험 값(stepper/homing/target_move_test.yaml) + coordinate.yaml을 읽는다."""
    coord = _load(os.path.join(cfg_dir, "coordinate.yaml"))
    stepper = _load(os.path.join(cfg_dir, "stepper_test.yaml"))
    homing = _load(os.path.join(cfg_dir, "homing_test.yaml"))
    target = _load(os.path.join(cfg_dir, "target_move_test.yaml"))
    need = {
        "stepper_test.yaml": {k: stepper.get(k) for k in ("positive_dir_level", "max_speed")},
        "homing_test.yaml": {k: homing.get(k) for k in (
            "fast_speed_steps_s", "slow_speed_steps_s", "backoff_steps", "max_seek_steps",
            "max_reseek_steps", "phase_timeout_s", "settle_s")},
        "target_move_test.yaml": {"speed_steps_s": target.get("speed_steps_s")},
    }
    missing = [f"{f} {k}" for f, vals in need.items() for k, v in vals.items() if v is None]
    if missing:
        raise ValueError("현장 시험 값이 비어 있음 (구동부 실측 후 채우기): " + ", ".join(missing))
    rail = coord["rail"]
    return {
        "geometry": DriveGeometry.from_config(coord),
        "travel_mm": float(rail["travel_mm"]),
        "deadzone_mm": float(coord.get("control", {}).get("deadzone_mm", 20)),
        "positive_dir_level": stepper["positive_dir_level"],
        "max_speed": stepper["max_speed"],
        "io_timeout": stepper.get("io_timeout") or 0.25,
        "homing": HomingConfig(**need["homing_test.yaml"], home_offset_mm=rail["home_offset_mm"]),
        "speed_steps_s": target["speed_steps_s"],
    }


def build_carriage_rail(pins, settings):
    """실장비용 CarriageRail 생성. 실패하면 이미 연 장치는 정리하고 예외를 그대로 던진다."""
    from carriage.hw_limit_switch import LimitSwitchHardware
    from carriage.hw_stepper import StepperHardware

    ls, st = pins["limit_switch"], pins["stepper"]
    limit_hw = LimitSwitchHardware(ls["left"], ls["right"])
    try:
        limits = LimitSwitchFunctions(limit_hw)
        hw = StepperHardware(st["step"], st["dir"], st["enable"],
                             positive_dir_level=settings["positive_dir_level"],
                             reserved_pins=tuple(ls.values()),
                             command_timeout_s=settings["io_timeout"])
    except Exception:
        limit_hw.cleanup()
        raise
    motor = StepperFunctions(hw, limits, max_speed_steps_s=settings["max_speed"])
    carriage = CarriageFunctions(motor, limits, settings["geometry"])
    homing = HomingProcedure(carriage, settings["homing"])
    mover = TargetMover(carriage, TargetMoveConfig(speed_steps_s=settings["speed_steps_s"],
                                                   min_mm=0, max_mm=settings["travel_mm"]))
    return CarriageRail(carriage, homing, mover, travel_mm=settings["travel_mm"],
                        deadzone_mm=settings["deadzone_mm"],
                        on_close=(carriage.stop, hw.cleanup, limit_hw.cleanup))
