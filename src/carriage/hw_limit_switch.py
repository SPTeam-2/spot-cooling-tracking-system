"""
[파일(내부 구현 코드) 목적]
Pi 4 양쪽 리미트 스위치의 GPIO 입력을 읽는 HW 계층
[코드 내용]
LimitSwitchHardware: BCM 핀 검증·내부 pull-up 설정·HIGH/LOW 읽기·핀 정리
배선: COM→GND, NO→GPIO / 눌림 해석·모터 제어: 상위 계층 담당
[플로우 간단 정리]
객체 생성 → 핀 검증·초기화 → read_levels()로 좌우 입력 반환 → cleanup()
Pi 4 리미트 스위치 HW 입력: COM→GND, NO→GPIO, NC 미사용

외부에서 BCM 핀 번호 전달 (프로젝트 설정: 왼쪽 5, 오른쪽 6)
눌림 해석·이동 차단·원점 설정: 상위 계층 담당
"""
from __future__ import annotations

from typing import Any


class LimitSwitchHardware:
    """내부 pull-up 설정과 GPIO 입력 담당

    gpio 생략 시 실제 RPi.GPIO 사용 / 시험 시 테스트용 가상 GPIO 전달
    같은 핀의 중복 소유·초기화 금지
    """

    def __init__(self, left_pin: int, right_pin: int, *, gpio: Any = None):
        for pin in (left_pin, right_pin):
            if type(pin) is not int or not 0 <= pin <= 27:
                raise ValueError("BCM 핀은 0~27 범위의 정수여야 합니다.")
        if left_pin == right_pin:
            raise ValueError("왼쪽과 오른쪽 GPIO 핀은 달라야 합니다.")
        if gpio is None:
            # PC에서 모듈만 import할 때 실제 GPIO 불필요
            import RPi.GPIO as gpio

        self._gpio = gpio
        self._pins = (left_pin, right_pin)
        self._closed = False
        gpio.setmode(gpio.BCM)
        configured = []
        try:
            for pin in self._pins:
                gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_UP)
                configured.append(pin)
        except Exception:
            # 부분 초기화 실패 시 먼저 설정한 핀만 정리 후 원래 오류 전달
            if configured:
                try:
                    gpio.cleanup(tuple(configured))
                except Exception:
                    pass  # 원래 초기화 오류 유지
            raise

    def read_levels(self) -> tuple[int, int]:
        """좌우 입력 (0=LOW, 1=HIGH) 즉시 반환

        두 핀 순차 읽기 / 완전한 동시 측정은 아님
        sleep·디바운스 없음 / GPIO 오류는 호출자에게 전달
        """
        if self._closed:
            raise RuntimeError("cleanup 이후에는 GPIO를 읽을 수 없습니다.")
        return (self._gpio.input(self._pins[0]), self._gpio.input(self._pins[1]))

    def cleanup(self) -> None:
        """스위치 핀만 정리 / 팬·모터 등 다른 핀 유지"""
        if not self._closed:
            self._gpio.cleanup(self._pins)
            self._closed = True
