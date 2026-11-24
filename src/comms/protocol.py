"""노트북(판단) ↔ 라즈베리파이(구동) 통신 프로토콜.

노트북이 매 프레임 Command를 UDP로 보내고, 파이는 최신값만 적용한다.
옛 명령은 버리므로 재연결 로직이 없고, 끊김은 파이쪽 워치독으로 처리한다.
파이는 명령을 보낸 주소로 Status(초음파 거리)를 돌려보낸다.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass

PORT = 5005
COMMS_TIMEOUT_S = 1.0    # 이 시간 넘게 명령 없으면 파이가 팬 정지
STATUS_PERIOD_S = 0.05   # 파이 → 노트북 Status 주기
STATUS_FRESH_S = 0.5     # 노트북이 이보다 오래된 Status는 무시


@dataclass
class Command:
    target_mm: float = 0.0   # 캐리지 목표 레일 위치
    fan_duty: int = 0        # 팬 세기 0~100%
    active: bool = False     # 송풍 대상 있음? False면 파이는 팬 정지
    ts: float = 0.0          # 보낸 시각(time.time())
    pan_deg: float = 0.0     # 팬틸트 좌우 (+ = 레일 + 방향, 모터에서 멀어지는 쪽)
    tilt_deg: float = 0.0    # 팬틸트 상하 (+ = 위)

    def to_bytes(self) -> bytes:
        return (json.dumps(asdict(self)) + "\n").encode("utf-8")

    @classmethod
    def from_bytes(cls, data: bytes) -> "Command":
        d = json.loads(data.decode("utf-8"))
        return cls(float(d["target_mm"]), int(d["fan_duty"]),
                   bool(d["active"]), float(d.get("ts", 0.0)),
                   float(d.get("pan_deg", 0.0)), float(d.get("tilt_deg", 0.0)))


@dataclass
class Status:
    sonar_mm: float | None = None   # 초음파 거리 (측정 없음 = None)
    ts: float = 0.0

    def to_bytes(self) -> bytes:
        return (json.dumps(asdict(self)) + "\n").encode("utf-8")

    @classmethod
    def from_bytes(cls, data: bytes) -> "Status":
        d = json.loads(data.decode("utf-8"))
        s = d.get("sonar_mm")
        return cls(None if s is None else float(s), float(d.get("ts", 0.0)))
