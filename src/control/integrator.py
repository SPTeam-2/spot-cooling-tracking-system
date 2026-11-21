"""인식 결과 → (레일 목표 mm, 팬 세기 %, 송풍 여부) 판단. 카메라·통신과 분리된 순수 로직."""

from __future__ import annotations

from src.control.depth import bbox_to_depth_mm, fuse_depth_mm
from src.control.priority import Worker, pick_target
from src.fan.fan_control import distance_to_duty

LOST_TIMEOUT_S = 2.0           # 이 시간 넘게 안 보이면 미검출
CARRIAGE_SPEED_MM_S = 300.0    # pi_server의 max_speed_mm_s와 같게 유지


class Integrator:
    def __init__(self, cfg: dict):
        ctrl, pr, fan = (cfg.get(k, {}) for k in ("control", "priority", "fan_power"))
        self.min_dwell_s = ctrl.get("min_dwell_s", 8)
        self.cooldown_s = ctrl.get("cooldown_s", 5)
        self.arrive_mm = ctrl.get("deadzone_mm", 20)
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
            step = CARRIAGE_SPEED_MM_S * dt
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
            return self.carriage_mm, 0, False

        o = self._obs[self.target.wid]
        size_px = o.shoulder_px if o.shoulder_px is not None else o.width_px   # 어깨 우선, 없으면 박스 폭
        self.depth_mm = fuse_depth_mm(bbox_to_depth_mm(size_px), sonar_mm)
        self.duty = distance_to_duty(self.depth_mm, self.duty, **self._fan)
        return o.rail_mm, self.duty, True
