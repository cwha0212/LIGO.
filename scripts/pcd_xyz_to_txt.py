#!/usr/bin/env python3
"""
PCD 파일을 읽어 x,y,z 좌표만 텍스트 파일로 저장한다.

주의: Open3D 가 읽는 **바이너리 PCD** 안의 좌표는 보통 float32 정도 정밀도만 갖는다.
그래서 ECEF 처럼 큰 수는 소수 부분이 .00 / .25 / .5 처럼 거칠게 보인다(스크립트 버그가 아님).
전자리가 필요하면 pcd_enu_to_ecef.py 의 --txt-output 으로 변환 직후 저장하거나,
원본을 ASCII/다른 포맷으로 보관할 것.

기본 출력은 유효숫자 17자리(%.17g)로, 메모리에 있는 값을 최대한 그대로 적는다.

의존성: pip install numpy open3d

사용 예:
  python3 scripts/pcd_xyz_to_txt.py PCD/scans_3.pcd -o PCD/scans_3_xyz.txt
  python3 scripts/pcd_xyz_to_txt.py map.pcd -o coords.csv --delimiter comma --header
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    print("open3d 가 필요합니다: pip install open3d", file=sys.stderr)
    raise SystemExit(1) from e


def main() -> None:
    ap = argparse.ArgumentParser(description="PCD → xyz 텍스트 저장")
    ap.add_argument("pcd", type=Path, help="입력 .pcd 경로")
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="출력 .txt (또는 .csv 등) 경로",
    )
    ap.add_argument(
        "--delimiter",
        choices=("space", "comma", "tab"),
        default="space",
        help="구분자: space(기본), comma, tab",
    )
    ap.add_argument(
        "--header",
        action="store_true",
        help="첫 줄에 x y z 헤더 작성",
    )
    ap.add_argument(
        "--precision",
        type=int,
        default=17,
        metavar="N",
        help="유효숫자 자릿수 (%%g 형식, float64 보존에는 17 권장, 기본 17)",
    )
    args = ap.parse_args()

    pcd_path = args.pcd.expanduser().resolve()
    out_path = args.output.expanduser().resolve()
    if not pcd_path.is_file():
        print(f"파일 없음: {pcd_path}", file=sys.stderr)
        sys.exit(1)

    pc = o3d.io.read_point_cloud(str(pcd_path))
    if pc.is_empty():
        print("포인트가 없거나 읽기 실패", file=sys.stderr)
        sys.exit(1)

    xyz = np.asarray(pc.points, dtype=np.float64)
    n = xyz.shape[0]

    delim = {"space": " ", "comma": ",", "tab": "\t"}[args.delimiter]
    # %g 유효숫자 — float64 텍스트 역변환에는 보통 17이면 충분(기본값)
    prec = max(1, args.precision)
    fmt = f"%.{prec}g"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        if args.header:
            f.write(f"x{delim}y{delim}z\n")
        np.savetxt(f, xyz, fmt=fmt, delimiter=delim)

    print(f"저장 완료: {out_path}  (포인트 {n}개)")


if __name__ == "__main__":
    main()
