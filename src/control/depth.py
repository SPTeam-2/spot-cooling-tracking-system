"""작업자 깊이(앞뒤 거리, mm) 추정 — 카메라 박스 크기 기반, 초음파는 보조.

핀홀 모델에서 깊이 ∝ 1 / 박스크기(px) → 기준점 하나로 보정해 역산한다.
박스는 '사람'에만 잡히므로 벽·사물을 거리로 오인하지 않는다(초음파 약점 보완).

크기(size_px)는 '가림에 강한 치수'를 넣는다 — 상체 높이나 어깨 폭 등.
전신 높이는 랙에 하체가 가리면 작아져 '멀다'고 오판하므로 피한다.
"""

from __future__ import annotations

# 보정 기준: 깊이 REF_DEPTH_MM일 때 사람 박스 크기가 REF_SIZE_PX.
# TODO: 실제 카메라로 아는 거리(예: 1m)에 사람 세워 박스 크기 측정해 갱신.
REF_DEPTH_MM = 1000.0
REF_SIZE_PX = 500.0


def bbox_to_depth_mm(size_px: float, *,
                     ref_depth_mm: float = REF_DEPTH_MM,
                     ref_size_px: float = REF_SIZE_PX) -> float:
    """사람 박스 크기(px) → 깊이(mm). 크기 클수록(가까울수록) 깊이 작아짐.

    size_px: 가림에 강한 치수(상체 높이/어깨 폭 등). ref_*도 같은 치수로 보정할 것.
    """
    if size_px <= 0:
        return float("inf")
    return ref_depth_mm * ref_size_px / size_px


def fuse_depth_mm(cam_mm: float, sonar_mm: float | None = None, *,
                  max_gap_mm: float = 400.0) -> float:
    """카메라 깊이(주) + 초음파(보조) 융합.

    카메라가 '사람 위치·대략 거리'를 신뢰값으로 잡는다. 초음파가 그와 비슷하면
    평균으로 미세보정하고, 크게 어긋나면(초음파가 벽·사물을 쟀다고 보고) 버린다.
    초음파 값이 없으면 카메라 값 그대로.
    """
    if sonar_mm is None:
        return cam_mm
    if abs(cam_mm - sonar_mm) > max_gap_mm:
        return cam_mm
    return (cam_mm + sonar_mm) / 2
