r"""실제 캘리브레이션: 화면에서 기준점 4개를 클릭해 호모그래피(화면 → 실제 mm) 생성

순서: 1(시작·바닥) → 2(끝·바닥) → 3(끝·위) → 4(시작·위)  ※ coordinate.yaml의 real_points 순서와 같음
조작: 왼쪽 클릭 = 점 찍기, r = 다시 찍기, Enter = 저장, q = 취소
저장 후 '검증 모드'로 바뀜: 아무 곳이나 클릭하면 그 점의 실제 좌표(mm)를 표시 → 자로 잰 값과 비교
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

COLORS = [(0, 0, 255), (0, 165, 255), (0, 255, 0), (255, 0, 0)]


def load_frame(args):
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise RuntimeError(f"이미지를 열 수 없음: {args.image}")
        return img
    cap = cv2.VideoCapture(args.index, cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for _ in range(10):          # 노출 안정될 때까지 몇 장 버림
        cap.read()
    ok, img = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("카메라 프레임을 읽을 수 없음")
    return img


def real_to_img(H_inv, pts):
    p = np.array(pts, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(p, H_inv).reshape(-1, 2)


def draw_grid(img, H, real_pts, step):
    """실제 좌표계 격자를 화면에 다시 그려서 캘리브레이션이 맞는지 눈으로 확인"""
    H_inv = np.linalg.inv(H)
    xs = [p[0] for p in real_pts]; zs = [p[1] for p in real_pts]
    x0, x1, z0, z1 = min(xs), max(xs), min(zs), max(zs)
    for x in np.arange(x0, x1 + 1e-6, step):
        a, b = real_to_img(H_inv, [(x, z0), (x, z1)])
        cv2.line(img, tuple(map(int, a)), tuple(map(int, b)), (255, 255, 0), 1)
    for z in np.arange(z0, z1 + 1e-6, step):
        a, b = real_to_img(H_inv, [(x0, z), (x1, z)])
        cv2.line(img, tuple(map(int, a)), tuple(map(int, b)), (255, 255, 0), 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--image", help="웹캠 대신 사진으로 캘리브레이션")
    ap.add_argument("--real", nargs=2, type=float, metavar=("W", "H"),
                    help="기준 사각형 실제 크기(mm). 생략하면 coordinate.yaml의 real_points 사용")
    ap.add_argument("--out", help="저장 경로. 생략하면 coordinate.yaml의 homography_file")
    ap.add_argument("--grid", type=float, default=None, help="검증 격자 간격(mm)")
    args = ap.parse_args()

    with open(os.path.join(ROOT, "config", "coordinate.yaml"), encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if args.real:
        w, h = args.real
        real_pts = [[0, 0], [w, 0], [w, h], [0, h]]
    else:
        real_pts = cfg["real_points"]
    out = args.out or cfg["homography_file"]
    out = out if os.path.isabs(out) else os.path.join(ROOT, out)
    grid = args.grid or max(real_pts[1][0], real_pts[2][1]) / 10

    base = load_frame(args)
    clicks, H, probe = [], None, []
    win = "calibrate (click 1-2-3-4, r: reset, Enter: save, q: quit)"

    def on_mouse(event, x, y, *_):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if H is None and len(clicks) < 4:
            clicks.append((x, y))
        elif H is not None:
            p = cv2.perspectiveTransform(np.array([[[x, y]]], np.float32), H)[0, 0]
            probe.append(((x, y), p))
            print(f"  클릭 ({x},{y})px → 실제 x={p[0]:.0f}mm, z={p[1]:.0f}mm")

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    cv2.setMouseCallback(win, on_mouse)
    print(f"기준점 실제 좌표(mm): {real_pts}")

    while True:
        img = base.copy()
        for i, (x, y) in enumerate(clicks):
            cv2.circle(img, (x, y), 7, COLORS[i], -1)
            cv2.putText(img, f"{i + 1} {real_pts[i]}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[i], 2)
        if H is not None:
            draw_grid(img, H, real_pts, grid)
            for (px, py), p in probe[-5:]:
                cv2.drawMarker(img, (px, py), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(img, f"x={p[0]:.0f} z={p[1]:.0f}mm", (px + 10, py + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            msg = "SAVED - click anywhere to check real mm (q: quit)"
        else:
            msg = f"click point {len(clicks) + 1}/4" if len(clicks) < 4 else "Enter: save / r: reset"
        cv2.putText(img, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imshow(win, img)

        key = cv2.waitKey(20) & 0xFF
        if key == ord("q"):
            break
        if key == ord("r"):
            clicks.clear(); probe.clear(); H = None
        if key in (13, 10) and len(clicks) == 4 and H is None:
            if not cv2.isContourConvex(np.array(clicks, np.int32).reshape(-1, 1, 2)):
                print("⚠ 점 순서가 엇갈렸어요. 테두리를 따라 1→2→3→4로 다시 찍으세요 (r)")
                clicks.clear()
                continue
            H, _ = cv2.findHomography(np.array(clicks, np.float32), np.array(real_pts, np.float32))
            back = cv2.perspectiveTransform(np.array(clicks, np.float32).reshape(-1, 1, 2), H).reshape(-1, 2)
            err = np.abs(back - np.array(real_pts)).max()
            os.makedirs(os.path.dirname(out), exist_ok=True)
            np.savez(out, H=H)
            print(f"저장: {out}  (기준점 재투영 오차 최대 {err:.2f}mm)")
            cv2.imwrite(out.replace(".npz", "_check.jpg"), img)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()