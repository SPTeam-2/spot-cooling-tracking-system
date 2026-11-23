"""노트북 송신부 — 인식·판단 결과를 파이로 전송(UDP).

연결 테스트: python3 src/comms/pc_sender.py --host <파이IP> --demo
고정 목표:   python3 src/comms/pc_sender.py --host <파이IP> --target 200   (--duty 팬 세기, --pan·--tilt 각도)
실제 통합:   python3 src/comms/pc_sender.py --host <파이IP>   (카메라 사용, --source로 영상 파일 가능)
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
from protocol import PORT, Command


class PiLink:
    def __init__(self, host: str, port: int = PORT):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, target_mm: float, fan_duty: int, active: bool,
             pan_deg: float = 0.0, tilt_deg: float = 0.0):
        cmd = Command(target_mm, fan_duty, active, time.time(), pan_deg, tilt_deg)
        self.sock.sendto(cmd.to_bytes(), self.addr)


def demo(link: PiLink, hz: int = 20):
    """카메라 없이 목표를 왕복시켜 파이와의 연결을 확인한다."""
    print(f"[노트북] 데모 송신 → {link.addr} (Ctrl+C 종료)")
    t0 = time.time()
    while True:
        t = time.time() - t0
        target = 300 + 300 * math.sin(t * 0.5)
        duty = 40 + int(60 * (0.5 + 0.5 * math.sin(t * 0.3)))
        pan = 15 * math.sin(t * 0.4)
        link.send(target, duty, True, pan, 0.0)
        time.sleep(1.0 / hz)


def hold(link: PiLink, target_mm: float, duty: int, pan: float = 0.0, tilt: float = 0.0,
         hz: int = 20):
    """같은 목표를 계속 보낸다 (캐리지 이동·팬틸트 시험용)."""
    print(f"[노트북] 고정 목표 {target_mm}mm, 팬 {duty}%, pan {pan}° tilt {tilt}° → {link.addr}"
          " (Ctrl+C 종료)")
    while True:
        link.send(target_mm, duty, True, pan, tilt)
        time.sleep(1.0 / hz)


def integration_loop(link: PiLink, source: str = "0", show: bool = True):
    """카메라 → 인식 → 대상 선정 → 깊이·팬 세기 → 파이 전송."""
    import cv2
    import yaml
    from src.control.integrator import Integrator
    from src.vision.perception import Perception

    os.chdir(ROOT)   # config 안의 상대경로(models/, data/) 기준
    with open("config/coordinate.yaml", encoding="utf-8") as f:
        integ = Integrator(yaml.safe_load(f))
    perception = Perception.from_config()
    if source.isdigit():
        cap = cv2.VideoCapture(int(source), cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)    # REF·캘리브레이션과 같은 해상도
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    else:
        cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"영상을 열 수 없음: {source}")

    print(f"[노트북] 통합 송신 → {link.addr} (q 또는 Ctrl+C 종료)")
    last_log = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            obs = perception.update(frame)
            now = time.time()
            target_mm, duty, active = integ.step(obs, now)
            link.send(target_mm, duty, active, integ.pan_deg, integ.tilt_deg)

            tid = integ.target.wid if active else None
            if now - last_log > 0.5:
                if active:
                    print(f"  대상 ID {tid} | 레일 {target_mm:5.0f}mm | 깊이 {integ.depth_mm:5.0f}mm"
                          f" | 팬 {duty:>3}% | pan {integ.pan_deg:+5.1f}° tilt {integ.tilt_deg:+5.1f}°"
                          f" | 보이는 사람 {len(obs)}명")
                else:
                    print(f"  대상 없음 | 보이는 사람 {len(obs)}명")
                last_log = now
            if show:
                for o in obs:
                    x1, y1, x2, y2 = o.bbox
                    hit = o.track_id == tid
                    color = (0, 0, 255) if hit else (0, 255, 0)
                    label = f"ID {o.track_id} | rail {o.rail_mm:.0f}mm"
                    if hit:
                        label += f" | depth {integ.depth_mm:.0f}mm | fan {duty}%"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 4 if hit else 2)
                    cv2.putText(frame, label, (x1, max(y1 - 10, 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.imshow("integration (q: quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        link.send(integ.carriage_mm, 0, active=False)
        cap.release()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="라즈베리파이 IP")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--demo", action="store_true", help="카메라 없이 연결 테스트")
    ap.add_argument("--target", type=float, help="카메라 없이 이 위치(mm)를 계속 보냄")
    ap.add_argument("--duty", type=int, default=0, help="--target과 함께 보낼 팬 세기(%%)")
    ap.add_argument("--pan", type=float, default=0.0, help="--target과 함께 보낼 pan 각도(도)")
    ap.add_argument("--tilt", type=float, default=0.0, help="--target과 함께 보낼 tilt 각도(도)")
    ap.add_argument("--source", default="0", help="카메라 번호 또는 영상 파일")
    ap.add_argument("--no-show", action="store_true", help="화면 창 없이 실행")
    args = ap.parse_args()

    link = PiLink(args.host, args.port)
    if args.demo or args.target is not None:
        try:
            if args.target is not None:
                hold(link, args.target, args.duty, args.pan, args.tilt)
            else:
                demo(link)
        except KeyboardInterrupt:
            link.send(0, 0, active=False)
    else:
        integration_loop(link, args.source, show=not args.no_show)


if __name__ == "__main__":
    main()
