"""
[파일(내부 구현 코드) 목적]
HW 입력을 좌우 눌림 상태·방향별 차단 여부로 해석하는 기능 계층
[코드 내용]
LimitSwitchState와 LimitSwitchFunctions: LOW=눌림 변환·동시 눌림·방향 검사
상태·판단값만 반환 / GPIO 직접 접근·모터 정지 명령 없음
[플로우 간단 정리]
HW 입력 읽기 → 입력값 검증 → 눌림 상태 반환 → 방향별 차단 여부 반환
양쪽 눌림 시 양방향 차단 / 입력 오류는 상위에 전달
GPIO 입력을 눌림·방향별 차단 여부로 해석

기준: COM-GND / NO-GPIO + 내부 pull-up / 눌림=LOW(0)

GPIO 직접 접근·모터 정지·좌표 설정·UDP 변경 없음
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class LimitSwitchReader(Protocol):
    """실제 HW·목업 공통 입력 인터페이스"""

    def read_levels(self) -> tuple[int, int]: ...


@dataclass(frozen=True)
class LimitSwitchState:
    """한 번 읽은 상태 / True=눌림, False=안 눌림"""

    left_pressed: bool
    right_pressed: bool

    @property
    def both_pressed(self) -> bool:
        """현재 한 축 구조의 입력 모순: 양쪽 동시 눌림"""
        return self.left_pressed and self.right_pressed


class LimitSwitchFunctions:
    """HW 객체를 받아 상태·차단 여부 제공 / 모터 구동 없음"""

    def __init__(self, hw: LimitSwitchReader):
        self._hw = hw

    def read_state(self) -> LimitSwitchState:
        """최신 입력 즉시 읽기 / 오류를 안 눌림으로 처리하지 않음

        NC 배선 변경 시 LOW=눌림 계약과 시험도 변경 필요
        안정 입력 확인: 모터 정지 후 상위 기능에서 별도 수행
        """
        left, right = self._hw.read_levels()
        if any(type(value) not in (int, bool) or value not in (0, 1)
               for value in (left, right)):
            raise ValueError("입력은 LOW(0) 또는 HIGH(1)여야 합니다.")
        return LimitSwitchState(left_pressed=left == 0, right_pressed=right == 0)

    def is_blocked(self, direction: int) -> bool:
        """최신 입력으로 차단 여부 판단 / -1=왼쪽, +1=오른쪽

        한쪽 눌림 시 반대 방향 후퇴 허용 / 양쪽 눌림 시 양방향 차단
        False: 리미트 차단 없음 / 전체 운전 허가를 뜻하지 않음
        STOP·원점·목표 범위는 상위에서 별도 확인 / STEP 직전에 호출
        """
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError("direction은 왼쪽 -1 또는 오른쪽 +1이어야 합니다.")
        state = self.read_state()
        return state.both_pressed or (
            state.left_pressed if direction == -1 else state.right_pressed
        )
