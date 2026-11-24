"""한 번에 실행: 랜선 확인 → SSH로 파이 서버 시작 → 원점 찾기 대기 → 노트북 송신.

실행 (Mac, conda activate sensorpro 후):
  python3 scripts/run_all.py --demo                  카메라 없이 왕복
  python3 scripts/run_all.py --target 300 --duty 70  고정 목표
  python3 scripts/run_all.py                         카메라 통합 (--source 1로 카메라 번호)
  --pull: 시작 전에 파이에서 git pull

연결은 랜선 고정: 노트북 192.168.1.10 ↔ 파이 192.168.1.9 (파이 eth0 고정 주소).
Ctrl+C 한 번이면 노트북 송신과 파이 서버를 함께 끈다 (파이는 팬·모터 정리 후 종료).
"""

from __future__ import annotations

import argparse
import os
import platform
import subprocess
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PI_DIR = "~/spot-cooling-tracking-system"
SERVER = "src/comms/pi_server.py"
SERVER_PATTERN = "^[p]ython3 .*pi_server"   # sudo가 아닌 python 프로세스만 (SIGINT 한 번만 받게)
HOMING_TIMEOUT_S = 300
READY_TAGS = ("[연결]", "[끊김/대기]")
FAIL_TAGS = ("오류:", "초기화 실패", "포트를 열 수 없음", "sudo:")
IMPORTANT = ("오류", "실패", "꺼짐", "sudo:")


def log(msg):
    print(f"[실행] {msg}", flush=True)


def is_mac():
    return platform.system() == "Darwin"


def ensure_lan(iface, my_ip):
    """Mac: 랜 어댑터에 고정 주소가 없으면 넣는다 (비밀번호 한 번). 다른 OS는 안내만."""
    if not is_mac():
        log(f"Mac이 아니라 주소 설정은 건너뜀 — 랜 어댑터 IPv4를 수동 {my_ip} / 255.255.255.0으로 해둘 것")
        return True
    out = subprocess.run(["ifconfig", iface], capture_output=True, text=True)
    if out.returncode != 0:
        log(f"랜 어댑터 {iface} 없음 — 랜선·어댑터 확인, 이름은 'networksetup -listallhardwareports'로 보고 --iface")
        return False
    if f"inet {my_ip} " in out.stdout:
        return True
    log(f"{iface}에 {my_ip} 넣는 중 (Mac 비밀번호)")
    return subprocess.run(["sudo", "ifconfig", iface, "alias", my_ip, "255.255.255.0"]).returncode == 0


def ping(host):
    cmd = ["ping", "-n", "1", "-w", "2000", host] if os.name == "nt" else ["ping", "-c", "1", "-t" if is_mac() else "-W", "2", host]
    return subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def ssh_cmd(pi, remote, tty=False):
    return ["ssh", "-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=5",
            *(["-tt"] if tty else []), pi, remote]


def remote_start(pull):
    """남은 서버 정리 → pigpiod·와이파이 절전 끄기 → 서버 실행.
    서버 경로를 변수로 넘겨 pkill 패턴이 이 SSH 셸 자신과 맞지 않게 한다."""
    pre = f"cd {PI_DIR}" + (" && git pull --ff-only" if pull else "")
    return (f"{pre} && S={SERVER}; sudo -n pkill -f '{SERVER_PATTERN}'; sleep 0.5; "
            "pgrep pigpiod >/dev/null || sudo -n pigpiod; sleep 0.5; "
            "sudo -n iw wlan0 set power_save off 2>/dev/null; "
            "exec sudo -n python3 -u $S")


def sender_args(a, pi_host):
    args = [sys.executable, os.path.join(ROOT, "src", "comms", "pc_sender.py"), "--host", pi_host]
    if a.demo:
        args.append("--demo")
    elif a.target is not None:
        args += ["--target", str(a.target), "--duty", str(a.duty), "--pan", str(a.pan), "--tilt", str(a.tilt)]
    else:
        args += ["--source", a.source] + (["--no-show"] if a.no_show else [])
    return args


class PiServer:
    """SSH로 띄운 pi_server의 출력을 읽어 상태를 판단한다."""

    def __init__(self, cmd):
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1,
                                     start_new_session=True)   # Ctrl+C가 SSH를 바로 죽이지 않게
        self.ready = threading.Event()
        self.failed = None
        self.quiet = False
        self._last_print = 0.0
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            line = line.rstrip()
            if not self.ready.is_set() and any(t in line for t in READY_TAGS):
                self.ready.set()
            if self.failed is None and any(t in line for t in FAIL_TAGS):
                self.failed = line.strip()
            now = time.time()
            if not self.quiet or any(t in line for t in IMPORTANT) or now - self._last_print >= 2.0:
                print(f"  [파이] {line.strip()}", flush=True)
                self._last_print = now
        if self.failed is None:
            self.failed = "파이 서버 연결이 끊김"

    def wait_ready(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.ready.wait(0.2):
                return True
            if self.failed or self.proc.poll() is not None:
                return False
        self.failed = f"{timeout}초 안에 원점 찾기가 끝나지 않음"
        return False


def stop_pi(pi, server):
    log("파이 서버 끄는 중 (팬·모터 정리)")
    subprocess.run(ssh_cmd(pi, f"sudo -n pkill -INT -f '{SERVER_PATTERN}'"),
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
    try:
        server.proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        server.proc.terminate()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="카메라 없이 0~600mm 왕복")
    ap.add_argument("--target", type=float, help="이 위치(mm)를 계속 보냄")
    ap.add_argument("--duty", type=int, default=70)
    ap.add_argument("--pan", type=float, default=0.0)
    ap.add_argument("--tilt", type=float, default=0.0)
    ap.add_argument("--source", default="0", help="카메라 번호 (아이폰이 0번을 잡으면 1)")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--pull", action="store_true", help="시작 전에 파이에서 git pull")
    ap.add_argument("--pi-ip", default="192.168.1.9")
    ap.add_argument("--my-ip", default="192.168.1.10")
    ap.add_argument("--iface", default="en22", help="Mac 랜 어댑터 이름")
    a = ap.parse_args()
    pi = f"pi@{a.pi_ip}"

    if not ensure_lan(a.iface, a.my_ip):
        sys.exit(1)
    if not ping(a.pi_ip):
        sys.exit(f"[실행] 파이 {a.pi_ip} 응답 없음 — 랜선, 파이 전원 확인")
    log(f"파이 {a.pi_ip} 연결됨 → 서버 시작")

    server = PiServer(ssh_cmd(pi, remote_start(a.pull)))
    sender = None
    try:
        log("원점 찾는 중 (최대 몇 분)")
        if not server.wait_ready(HOMING_TIMEOUT_S):
            log(f"파이 서버 시작 실패: {server.failed}")
            return
        server.quiet = True
        log("원점 완료 → 노트북 송신 시작 (Ctrl+C: 모두 종료)")
        sender = subprocess.Popen(sender_args(a, a.pi_ip))
        while sender.poll() is None:
            if server.proc.poll() is not None:
                log(f"파이 서버가 멈춤: {server.failed}")
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if sender and sender.poll() is None:
            sender.terminate()
            try:
                sender.wait(timeout=3)
            except subprocess.TimeoutExpired:
                sender.kill()
        if server.proc.poll() is None:
            stop_pi(pi, server)
        log("종료")


if __name__ == "__main__":
    main()
