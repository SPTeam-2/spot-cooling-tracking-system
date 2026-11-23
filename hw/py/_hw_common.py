"""hw/py/ 점검 스크립트 공통 모듈.

목적: 하드웨어 점검 스크립트들이 같은 핀 번호와 같은 환산식을 쓰도록 모아 둔다.
내용: config/pins.yaml 읽기, pigpiod 연결 확인, 스텝·mm 환산.
흐름: 점검 스크립트 → load_pins()로 핀 확인 → connect()로 pigpiod 연결 → 환산 함수로 주파수 계산.

Pi에서만 동작한다 (pigpio 필요). 실행 전 `pgrep pigpiod || sudo pigpiod`.
"""

from __future__ import annotations

import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))   # hw/py/ → 저장소 루트
PINS_PATH = os.path.join(ROOT, "config", "pins.yaml")

FULL_STEPS_PER_REV = 200   # NEMA17 1.8도
MM_PER_REV = 40            # GT2 2mm 피치 × 20T 풀리

# pigpio 기본 설정(샘플링 5µs)에서 set_PWM_frequency로 쓸 수 있는 주파수 일부
PWM_ALLOWED_HZ = [10, 20, 40, 50, 80, 100, 160, 200, 250, 320, 400, 500, 800, 1000, 1600, 2000]

# 가속 시작 주파수. 현장 점검(2026-11-22~23)에서 160Hz부터 단계 가속했을 때 잘 움직였음
RAMP_START_HZ = 160


def ramp_steps(top_hz: int) -> list[int]:
    """top_hz보다 낮은 허용 주파수 중 RAMP_START_HZ 이상만 가속 단계로 쓴다."""
    return [f for f in PWM_ALLOWED_HZ if RAMP_START_HZ <= f < top_hz]


def load_pins() -> dict:
    """config/pins.yaml을 읽어 그대로 돌려준다 (BCM 번호)."""
    try:
        import yaml
    except ImportError as exc:
        raise SystemExit("PyYAML이 필요합니다: pip install pyyaml") from exc
    with open(PINS_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


def connect():
    """pigpio 모듈과 연결된 pi 객체를 돌려준다. 실패하면 안내 후 종료."""
    try:
        import pigpio
    except ImportError as exc:
        raise SystemExit("pigpio 모듈이 없습니다. Raspberry Pi에서 실행하세요.") from exc
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpiod 연결 실패: sudo pigpiod 먼저 실행")
    return pigpio, pi


def steps_per_rev(micro: int) -> int:
    return FULL_STEPS_PER_REV * micro


def steps_per_mm(micro: int) -> float:
    return steps_per_rev(micro) / MM_PER_REV
