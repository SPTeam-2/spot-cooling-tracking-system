"""송풍 출력 제어 — 작업자 깊이(mm) → 팬 PWM 세기(%) 매핑 + GPIO PWM 구동.

distance_to_duty()는 순수 로직(단독 테스트 가능), FanController는 파이에서만 동작.
"""

from __future__ import annotations


def distance_to_duty(
    dist_mm: float,
    prev_duty: int | None = None,
    *,
    near_max_mm: float = 500,
    mid_max_mm: float = 1200,
    duty_near: int = 40,
    duty_mid: int = 70,
    duty_far: int = 100,
    hysteresis_mm: float = 50,
) -> int:
    """깊이(mm) → PWM 세기(%). 미풍 / 약풍 / 강풍 3단계.

    prev_duty를 주면 경계에서의 잦은 단계 전환(떨림)을 히스테리시스로 억제.
    """
    near = near_max_mm
    mid = mid_max_mm
    # 이전 단계보다 더 강한 단계로 올라갈 땐 경계를 조금 더 멀리 둔다 (올라가기 어렵게)
    if prev_duty is not None:
        if prev_duty <= duty_near:
            near += hysteresis_mm
            mid += hysteresis_mm
        elif prev_duty <= duty_mid:
            mid += hysteresis_mm

    if dist_mm <= near:
        return duty_near
    if dist_mm <= mid:
        return duty_mid
    return duty_far


class FanController:
    """MOSFET(D4184) 게이트를 PWM으로 구동해 팬 세기를 제어 (파이에서만)."""

    def __init__(self, pin: int, freq_hz: int = 100):
        import RPi.GPIO as GPIO  # 파이에서만 import (맥에선 사용 안 함)

        self._GPIO = GPIO
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, freq_hz)
        self._pwm.start(0)
        self._duty = 0

    def set_duty(self, duty: int) -> None:
        """0 또는 40~100(%)로 설정. 0은 완전 정지."""
        duty = max(0, min(100, int(duty)))
        self._pwm.ChangeDutyCycle(duty)
        self._duty = duty

    def off(self) -> None:
        self.set_duty(0)

    def cleanup(self) -> None:
        self._pwm.stop()
        self._GPIO.cleanup(self.pin)
