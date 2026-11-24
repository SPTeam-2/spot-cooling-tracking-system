"""인식 결과 → (레일 목표 mm, 팬 세기 %, 송풍 여부) 판단. 카메라·통신과 분리된 순수 로직."""

from __future__ import annotations

import math

from src.control.depth import bbox_to_depth_mm, fuse_depth_mm
from src.control.priority import Worker, pick_target
from src.fan.fan_control import distance_to_duty

LOST_TIMEOUT_S = 2.0           # 이 시간 넘게 안 보이면 미검출
CARRIAGE_SPEED_MM_S = 300.0    # coordinate.yaml control.carriage_speed_mm_s가 없을 때 기본값


class Integrator:
    def __init__(self, cfg: dict):
        ctrl, pr, fan, dep = (cfg.get(k) or {} for k in ("control", "priority", "fan_power", "depth"))
        self.min_dwell_s = ctrl.get("min_dwell_s", 8)
        self.cooldown_s = ctrl.get("cooldown_s", 5)
        self.arrive_mm = ctrl.get("deadzone_mm", 20)
        self.speed_mm_s = ctrl.get("carriage_speed_mm_s", CARRIAGE_SPEED_MM_S)
        self._pt = cfg.get("pan_tilt") or {}
        self._fuse = {k: dep[k] for k in ("sonar_weight", "max_gap_mm") if k in dep}
        self.pan_deg, self.tilt_deg = self._fixed_angles()
        self._pr = {k: pr[k] for k in ("cooled_similar_s", "wait_similar_s") if k in pr}
        self._fan = {k: fan[k] for k in ("near_max_mm", "mid_max_mm", "duty_near", "duty_mid",
                                         "duty_far", "hysteresis_mm") if k in fan}
        lo, hi = cfg.get("rail_range", [0, 600])
        self.carriage_mm = (lo + hi) / 2   # 추정값 (파이에서 위치를 돌려받지 않음)
        self.workers: dict[int, Worker] = {}
        self.target: Worker | None = None
        self.depth_mm: float | None = None
        self.duty: int | None = None
        self._obs = {}
        self._last_seen: dict[int, float] = {}
        self._dwell_left = 0.0
        self._t_prev: float | None = None

    def step(self, obs, now: float, sonar_mm: float | None = None) -> tuple[float, int, bool]:
        dt = 0.0 if self._t_prev is None else now - self._t_prev
        self._t_prev = now

        for o in obs:
            self.workers.setdefault(o.track_id, Worker(wid=o.track_id))
            self._obs[o.track_id] = o
            self._last_seen[o.track_id] = now
        for tid, w in self.workers.items():
            w.detected = now - self._last_seen[tid] < LOST_TIMEOUT_S
            w.distance_mm = abs(self._obs[tid].rail_mm - self.carriage_mm)

        if self.target is None or self._dwell_left <= 0 or not self.target.detected:
            if self.target is not None and self._dwell_left <= 0:
                self.target.cooldown_until = now + self.cooldown_s
            ws = list(self.workers.values())
            new = pick_target(ws, now, **self._pr)
            if new is None:   # 전원 쿨다운 중이면 쿨다운 무시 (혼자일 때 팬이 꺼지지 않게)
                new = pick_target(ws, float("inf"), **self._pr)
            if new is not self.target:
                self.target, self.duty = new, None
            self._dwell_left = self.min_dwell_s

        arrived = False
        if self.target is not None:
            goal = self._obs[self.target.wid].rail_mm
            step = self.speed_mm_s * dt
            diff = goal - self.carriage_mm
            self.carriage_mm = goal if abs(diff) <= step else self.carriage_mm + (step if diff > 0 else -step)
            arrived = abs(goal - self.carriage_mm) < self.arrive_mm
        for w in self.workers.values():
            if not w.detected:
                continue
            if w is self.target and arrived:
                w.cooled_total_s += dt
                w.wait_s = 0
            else:
                w.wait_s += dt
        if arrived:
            self._dwell_left -= dt

        if self.target is None:
            self.depth_mm = self.duty = None
            self.pan_deg, self.tilt_deg = self._fixed_angles()
            return self.carriage_mm, 0, False

        o = self._obs[self.target.wid]
        size_px = o.shoulder_px if o.shoulder_px is not None else o.width_px   # 어깨 우선, 없으면 박스 폭
        self.depth_mm = fuse_depth_mm(bbox_to_depth_mm(size_px), sonar_mm, **self._fuse)
        self.duty = distance_to_duty(self.depth_mm, self.duty, **self._fan)
        self.pan_deg, self.tilt_deg = self._aim(o.rail_mm, self.depth_mm)
        return o.rail_mm, self.duty, True

    def _fixed_angles(self) -> tuple[float, float]:
        return float(self._pt.get("fixed_pan_deg", 0)), float(self._pt.get("fixed_tilt_deg", 0))

    def _aim(self, goal_mm: float, depth_mm: float) -> tuple[float, float]:
        """auto: 캐리지가 아직 못 따라간 만큼 pan으로, 사람 높이와 거리로 tilt를 맞춤."""
        if self._pt.get("mode") != "auto" or not math.isfinite(depth_mm) or depth_mm <= 0:
            return self._fixed_angles()
        pt = self._pt
        pan = math.degrees(math.atan2(goal_mm - self.carriage_mm, depth_mm))
        tilt = math.degrees(math.atan2(pt.get("aim_height_mm", 1200) - pt.get("fan_height_mm", 900),
                                       depth_mm))
        lim_p, lim_t = pt.get("max_pan_deg", 30), pt.get("max_tilt_deg", 30)
        return max(-lim_p, min(lim_p, pan)), max(-lim_t, min(lim_t, tilt))
