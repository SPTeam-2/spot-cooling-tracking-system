"""송풍 출력 제어

거리(cm) → 팬 PWM 세기(%) 3단계 매핑 + 실제 GPIO PWM 제어.
멀수록 강풍, 가까우면 미풍. 경계 떨림 방지를 위한 히스테리시스 포함.

- distance_to_duty(): 순수 로직 → 하드웨어 없이 단독 테스트 가능
- FanController       : RPi.GPIO로 MOSFET(D4184) PWM 제어 (파이에서만 동작)
"""

from __future__ import annotations


def distance_to_duty(
    dist_cm: float,
    prev_duty: int | None = None,
    *,
    near_max_cm: float = 50,
    mid_max_cm: float = 120,
    duty_near: int = 40,
    duty_mid: int = 70,
    duty_far: int = 100,
    hysteresis_cm: float = 5,
) -> int:
    """거리(cm) → PWM 세기(%). 미풍 / 약풍 / 강풍 3단계.

    prev_duty를 주면 경계에서의 잦은 단계 전환(떨림)을 히스테리시스로 억제.
    """
    near = near_max_cm
    mid = mid_max_cm
    # 이전 단계보다 더 강한 단계로 올라갈 땐 경계를 조금 더 멀리 둔다 (올라가기 어렵게)
    if prev_duty is not None:
        if prev_duty <= duty_near:
            near += hysteresis_cm
            mid += hysteresis_cm
        elif prev_duty <= duty_mid:
            mid += hysteresis_cm

    if dist_cm <= near:
        return duty_near
    if dist_cm <= mid:
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


if __name__ == "__main__":
    # 하드웨어 없이 매핑 로직만 테스트
    print("거리(cm) → 송풍 세기(%)")
    prev = None
    for d in [30, 48, 52, 55, 100, 118, 125, 200]:
        duty = distance_to_duty(d, prev)
        label = {40: "미풍", 70: "약풍", 100: "강풍"}[duty]
        print(f"  {d:>4}cm → {duty:>3}% ({label})")
        prev = duty
