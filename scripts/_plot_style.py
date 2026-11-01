"""그래프 공통 스타일 — OS별 한글 폰트 자동 선택 (맥/윈도우/리눅스 호환).

맥은 AppleGothic, 윈도우는 Malgun Gothic, 리눅스는 NanumGothic 중
설치된 첫 폰트를 사용. 윈도우에서 한글이 깨지던 문제 해결.
"""

from __future__ import annotations


def set_korean_font(plt):
    import matplotlib.font_manager as fm

    avail = {f.name for f in fm.fontManager.ttflist}
    for name in ["AppleGothic", "Malgun Gothic", "NanumGothic", "NanumGothicCoding"]:
        if name in avail:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False
