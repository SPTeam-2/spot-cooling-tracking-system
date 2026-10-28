"""실제 좌표 → 레일 좌표 비율 변환

레일 좌표 = 레일 시작 + (실제 x - 실제 시작) × (레일 길이 ÷ 실제 길이)
레일 범위를 벗어나면 양 끝값으로 자름 (클램핑).
"""


class RailScaler:
    def __init__(self, real_range, rail_range):
        self.real_min, self.real_max = real_range
        self.rail_min, self.rail_max = rail_range
        if self.real_max <= self.real_min or self.rail_max <= self.rail_min:
            raise ValueError("범위는 [최소, 최대] 순서이고 최대가 더 커야 합니다.")
        self.scale = (self.rail_max - self.rail_min) / (self.real_max - self.real_min)

    def to_rail(self, x_real):
        """실제 x(mm) → 레일 목표 좌표(mm)."""
        x_rail = self.rail_min + (x_real - self.real_min) * self.scale
        return max(self.rail_min, min(self.rail_max, x_rail))