"""노트북(판단) ↔ 라즈베리파이(구동) 통신 프로토콜.

노트북이 매 프레임 Command를 UDP로 보내고, 파이는 최신값만 적용한다.
옛 명령은 버리므로 재연결 로직이 없고, 끊김은 파이쪽 워치독으로 처리한다.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass

PORT = 5005
COMMS_TIMEOUT_S = 1.0    # 이 시간 넘게 명령 없으면 파이가 팬 정지


@dataclass
class Command:
    target_mm: float = 0.0   # 캐리지 목표 레일 위치
    fan_duty: int = 0        # 팬 세기 0~100%
    active: bool = False     # 송풍 대상 있음? False면 파이는 팬 정지
    ts: float = 0.0          # 보낸 시각(time.time())

    def to_bytes(self) -> bytes:
        return (json.dumps(asdict(self)) + "\n").encode("utf-8")

    @classmethod
    def from_bytes(cls, data: bytes) -> "Command":
        d = json.loads(data.decode("utf-8"))
        return cls(float(d["target_mm"]), int(d["fan_duty"]),
                   bool(d["active"]), float(d.get("ts", 0.0)))
