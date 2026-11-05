"""노트북 송신부 — 인식·판단 결과를 파이로 전송(UDP).

실행(연결 테스트): python3 src/comms/pc_sender.py --host <파이IP> --demo
실제 통합은 integration_loop() 패턴대로 매 프레임 link.send(...)를 호출한다.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from protocol import PORT, Command


class PiLink:
    def __init__(self, host: str, port: int = PORT):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, target_mm: float, fan_duty: int, active: bool):
        cmd = Command(target_mm, fan_duty, active, time.time())
        self.sock.sendto(cmd.to_bytes(), self.addr)


def demo(link: PiLink, hz: int = 20):
    """카메라 없이 목표를 왕복시켜 파이와의 연결을 확인한다."""
    print(f"[노트북] 데모 송신 → {link.addr} (Ctrl+C 종료)")
    t0 = time.time()
    while True:
        t = time.time() - t0
        target = 300 + 300 * math.sin(t * 0.5)
        duty = 40 + int(60 * (0.5 + 0.5 * math.sin(t * 0.3)))
        link.send(target, duty, active=True)
        time.sleep(1.0 / hz)


def integration_loop(link: PiLink):
    """실제 통합 뼈대 (카메라·부품 준비 후 아래 패턴대로 구현)."""
    raise NotImplementedError

    # from vision.tracker import PersonTracker
    # from coordinate.calibration import CoordinateTransformer
    # from control.priority import Worker, pick_target
    # from control.depth import bbox_to_depth_mm, fuse_depth_mm
    # from fan.fan_control import distance_to_duty
    #
    # tracker = PersonTracker.from_config()
    # transformer = CoordinateTransformer.from_config()
    # workers, carriage_mm = {}, 300.0
    # while True:
    #     dets = tracker.track(frame)
    #     seen = []
    #     for d in dets:
    #         _, rail_mm = transformer.transform(d.ref_point)   # 레일 위치(좌우)
    #         w = workers.setdefault(d.track_id, Worker(wid=d.track_id))
    #         w.distance_mm = abs(rail_mm - carriage_mm)         # 캐리지 상대거리(선정용)
    #         w._rail_mm = rail_mm
    #         # 깊이용 크기: 전신 높이는 하체 가림에 약함 → 상체 높이/폭 등으로 교체 가능
    #         w._size_px = d.bbox[3] - d.bbox[1]
    #         seen.append(d.track_id)
    #     for tid, w in workers.items():
    #         w.detected = tid in seen
    #
    #     target = pick_target(list(workers.values()), now=time.time())
    #     if target is None:
    #         link.send(carriage_mm, 0, active=False)
    #     else:
    #         # 팬 세기는 레일거리가 아니라 '깊이(앞뒤)'로 — 카메라 크기(주) + 초음파(보조)
    #         depth_mm = fuse_depth_mm(bbox_to_depth_mm(target._size_px), sonar_mm=None)
    #         duty = distance_to_duty(depth_mm)
    #         # carriage_mm은 추정값(단방향이라 실제 위치 피드백 없음). 3순위 선정에만 쓰여 영향 작음
    #         carriage_mm = target._rail_mm
    #         link.send(carriage_mm, duty, active=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="라즈베리파이 IP")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--demo", action="store_true", help="카메라 없이 연결 테스트")
    args = ap.parse_args()

    link = PiLink(args.host, args.port)
    if args.demo:
        try:
            demo(link)
        except KeyboardInterrupt:
            link.send(0, 0, active=False)
    else:
        integration_loop(link)


if __name__ == "__main__":
    main()
