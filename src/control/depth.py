"""작업자 깊이(앞뒤 거리, mm) 추정 — 카메라 bbox 높이 기반, 초음파는 보조.

핀홀 모델에서 깊이 ∝ 1 / 박스높이(px) → 기준점 하나로 보정해 역산한다.
박스는 '사람'에만 잡히므로 벽·사물을 거리로 오인하지 않는다(초음파 약점 보완).
"""

from __future__ import annotations

# 보정 기준: 깊이 REF_DEPTH_MM일 때 사람 박스 높이가 REF_BBOX_PX.
# TODO: 실제 카메라로 아는 거리(예: 1m)에 사람 세워 박스 높이 측정해 갱신.
REF_DEPTH_MM = 1000.0
REF_BBOX_PX = 500.0


def bbox_to_depth_mm(bbox_h_px: float, *,
                     ref_depth_mm: float = REF_DEPTH_MM,
                     ref_bbox_px: float = REF_BBOX_PX) -> float:
    """사람 박스 높이(px) → 깊이(mm). 박스 클수록(가까울수록) 깊이 작아짐."""
    if bbox_h_px <= 0:
        return float("inf")
    return ref_depth_mm * ref_bbox_px / bbox_h_px


def fuse_depth_mm(cam_mm: float, sonar_mm: float | None = None, *,
                  max_gap_mm: float = 400.0) -> float:
    """카메라 깊이(주) + 초음파(보조) 융합.

    둘이 크게 어긋나면(박스 추정이 틀어졌다고 보고) 실측 초음파를 쓰고,
    가까우면 평균으로 안정화. 초음파 값이 없으면 카메라 값 그대로.
    """
    if sonar_mm is None:
        return cam_mm
    if abs(cam_mm - sonar_mm) > max_gap_mm:
        return sonar_mm
    return (cam_mm + sonar_mm) / 2
