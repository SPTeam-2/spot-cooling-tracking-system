from __future__ import annotations

# 보정 기준: 1000mm 거리에서 박스 폭 250px. 실측 후 갱신 필요.
REF_DEPTH_MM = 1000.0
REF_SIZE_PX = 250.0


def bbox_to_depth_mm(size_px: float, *,
                     ref_depth_mm: float = REF_DEPTH_MM,
                     ref_size_px: float = REF_SIZE_PX) -> float:
    if size_px <= 0:
        return float("inf")
    return ref_depth_mm * ref_size_px / size_px


def fuse_depth_mm(cam_mm: float, sonar_mm: float | None = None, *,
                  max_gap_mm: float = 400.0) -> float:
    # 초음파가 max_gap 이상 벗어나면 벽·사물로 보고 버림
    if sonar_mm is None:
        return cam_mm
    if abs(cam_mm - sonar_mm) > max_gap_mm:
        return cam_mm
    return (cam_mm + sonar_mm) / 2
