"""
[파일(내부 구현 코드) 목적]
GT2 사양 검증과 거리·스텝 환산 규칙을 실제 장치 없이 자동 검증
[코드 내용]
GeometryTests: 예시 사양의 steps_per_mm, 양·음수 거리, 반 스텝 반올림, 0스텝 환산 검사
잘못된 사양·NaN·무한값·잘못된 거리 타입 거절 검사
[플로우 간단 정리]
시험 사양 생성 → 거리·스텝 환산 → 기대값·오차 확인 → 잘못된 입력의 예외 확인

예시 사양의 계산 검사이며 실제 풀리 잇수·MS 배선·이동 거리 검증은 별도 필요
"""
import unittest
from src.carriage.drive_geometry import DriveGeometry


class GeometryTests(unittest.TestCase):
    def test_conversion_and_rounding(self):
        g = DriveGeometry(2, 20, 200, 16)
        self.assertEqual(g.steps_per_mm, 80)
        for mm, steps in ((10, 800), (-10, -800), (0, 0), (0.001, 0),
                          (0.00625, 1), (-0.00625, -1)):
            self.assertEqual(g.mm_to_steps(mm), steps)
            self.assertLessEqual(abs(g.steps_to_mm(steps)-mm), 1/160)

    def test_invalid_specs(self):
        for args in ((0,20,200,16), (float('nan'),20,200,16), (2,True,200,16),
                     (2,20,0,16), (2,20,200,3), (2,20,200,True)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                DriveGeometry(*args)

    def test_invalid_distance(self):
        for value in (float('nan'), float('inf'), True, '1'):
            with self.assertRaises(ValueError):
                DriveGeometry(2,20,200,16).mm_to_steps(value)
