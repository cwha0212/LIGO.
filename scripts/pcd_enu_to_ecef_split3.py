#!/usr/bin/env python3
"""
PCD(ENU) → (선택) z 필터 → XY 평면에서 한 축 기준 3등분 → 구간별 voxel → ECEF → PCD 3개 저장.

pcd_enu_to_ecef.py 와 동일한 gnss_comm 정의·YAML 처리·옵션을 유지한다.
3등분: 최초 ENU에서 지정 축(x 또는 y)의 [min, max] 구간을 길이로 동일 3분할한다.

의존성: pip install numpy open3d pyyaml

예:
  python3 scripts/pcd_enu_to_ecef_split3.py PCD/scans_3.pcd PCD/scans_3_grid2d.yaml \
      --split-axis y --voxel 0.5 -o PCD/scans_3_ecef_split.pcd --enu-z-max 2.5
  → PCD/scans_3_ecef_split_part1.pcd, _part2.pcd, _part3.pcd

  # 위에서 보는 맵에서 '가로'로 자르려면 보통 y 축 3등분 (--split-axis y).
  # 세로 띠는 --split-axis x.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    print("open3d 가 필요합니다: pip install open3d", file=sys.stderr)
    raise SystemExit(1) from e

# 동일 디렉터리의 pcd_enu_to_ecef.py 를 모듈로 로드 (실행 cwd 무관)
_SCRIPT_DIR = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    "pcd_enu_to_ecef", _SCRIPT_DIR / "pcd_enu_to_ecef.py"
)
if _spec is None or _spec.loader is None:
    raise RuntimeError("pcd_enu_to_ecef.py 를 불러올 수 없습니다.")
_pe = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pe)


def _read_point_cloud(path: Path) -> o3d.geometry.PointCloud:
    pc = o3d.io.read_point_cloud(str(path))
    if pc.is_empty():
        raise ValueError(f"포인트가 없거나 읽기 실패: {path}")
    return pc


def _axis_index(split_axis: str) -> int:
    a = split_axis.lower().strip()
    if a == "x":
        return 0
    if a == "y":
        return 1
    raise ValueError("split_axis 는 x 또는 y 여야 합니다.")


def masks_enu_axis_thirds(points: np.ndarray, axis_idx: int) -> Tuple[List[np.ndarray], np.ndarray]:
    """
    ENU points (N,3)에서 axis_idx 축으로 [min,max]를 동일 길이 3구간으로 나눈다.
    반환: 길이 3의 bool 마스크 리스트, edges 길이 4 (linspace).
    경계는 [e0,e1), [e1,e2), [e2,e3] (마지막 구간은 e3 포함).
    """
    v = points[:, axis_idx].astype(np.float64)
    vmin = float(v.min())
    vmax = float(v.max())
    if not np.isfinite(vmin) or not np.isfinite(vmax) or vmax <= vmin:
        raise ValueError(f"축 {axis_idx} 유효 범위가 없습니다 (min={vmin}, max={vmax}).")
    edges = np.linspace(vmin, vmax, 4)
    masks: List[np.ndarray] = []
    for i in range(3):
        lo, hi = float(edges[i]), float(edges[i + 1])
        if i < 2:
            m = (v >= lo) & (v < hi)
        else:
            m = (v >= lo) & (v <= hi)
        masks.append(m)
    return masks, edges


def _stem_outputs(base: Path, n: int) -> List[Path]:
    """base = path/to/name.pcd -> name_part1.pcd ..."""
    parent = base.parent
    stem = base.stem
    suf = base.suffix if base.suffix else ".pcd"
    return [parent / f"{stem}_part{k}{suf}" for k in range(1, n + 1)]


def main() -> None:
    ap = argparse.ArgumentParser(
        description="PCD ENU → XY 한 축 3등분 → 구간별 ECEF PCD 저장 (pcd_enu_to_ecef 와 동일 변환)"
    )
    ap.add_argument("pcd", type=Path, help="입력 PCD (ENU, x/y/z)")
    ap.add_argument(
        "grid_yaml",
        type=Path,
        help="*_grid2d.yaml (ecef_from_enu 포함)",
    )
    ap.add_argument(
        "--split-axis",
        choices=("x", "y"),
        default="y",
        help="ENU XY 평면에서 이 축 방향으로 범위를 3등분 (위에서 가로 띠: y, 세로 띠: x). 기본 y",
    )
    ap.add_argument(
        "--rotation",
        choices=("gnss_comm", "yaml", "lla"),
        default="gnss_comm",
        help="R 계산 방식 (pcd_enu_to_ecef.py 와 동일)",
    )
    ap.add_argument(
        "--enu-z-max",
        type=float,
        default=None,
        metavar="M",
        help="ENU z≥M (m) 제거 후 3등분·voxel·ECEF",
    )
    ap.add_argument(
        "--voxel",
        type=float,
        default=0.5,
        metavar="M",
        help="구간별 VoxelLeaf (m). 0 이하면 다운샘플 생략",
    )
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="출력 PCD 베이스 경로 (확장자 포함 권장). _part1/_part2/_part3 가 붙는다.",
    )
    ap.add_argument(
        "--txt-output",
        type=Path,
        default=None,
        metavar="PATH",
        help="베이스 텍스트 경로 (예: out.txt → out_part1.txt …). 미지정 시 txt 없음",
    )
    ap.add_argument(
        "--txt-precision",
        type=int,
        default=17,
        metavar="N",
        help="txt 유효숫자 (기본 17)",
    )
    args = ap.parse_args()

    pcd_path = args.pcd.expanduser().resolve()
    yaml_path = args.grid_yaml.expanduser().resolve()
    if not pcd_path.is_file():
        print(f"파일 없음: {pcd_path}", file=sys.stderr)
        sys.exit(1)
    if not yaml_path.is_file():
        print(f"파일 없음: {yaml_path}", file=sys.stderr)
        sys.exit(1)

    meta = _pe.load_ecef_block_meta(yaml_path)
    anchor = meta["anchor_ecef"]
    r_ecef_enu, r_desc = _pe.resolve_r_ecef_enu(args.rotation, meta)
    axis_idx = _axis_index(args.split_axis)
    ax_name = "x" if axis_idx == 0 else "y"

    pc = _read_point_cloud(pcd_path)
    pts = np.asarray(pc.points, dtype=np.float64)
    n0 = pts.shape[0]

    if args.enu_z_max is not None:
        z_th = float(args.enu_z_max)
        mask_z = pts[:, 2] < z_th
        n_kept = int(mask_z.sum())
        if n_kept == 0:
            raise ValueError(
                f"ENU z<{z_th} m 조건으로 포인트가 모두 제거되었습니다. (--enu-z-max 조정)"
            )
        pts = pts[mask_z]
        cols = np.asarray(pc.colors) if pc.has_colors() else None
        if cols is not None and len(cols) == n0:
            cols = cols[mask_z]
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts)
        if cols is not None:
            pc.colors = o3d.utility.Vector3dVector(cols)
        print(
            f"[0] ENU z 필터: z≥{z_th:g} m 제거 → {n0} → {n_kept} 포인트 "
            f"(남은 z∈[{pts[:, 2].min():.12g}, {pts[:, 2].max():.12g}] m)"
        )
        n0 = n_kept

    pts = np.asarray(pc.points, dtype=np.float64)
    xy_min, xy_max = _pe._xy_range(pts)
    z_lo, z_hi = pts[:, 2].min(), pts[:, 2].max()
    print(f"[1] 입력: {pcd_path.name}  포인트 = {n0}")
    print(f"    ENU x∈[{xy_min[0]:.12g}, {xy_max[0]:.12g}]  y∈[{xy_min[1]:.12g}, {xy_max[1]:.12g}]  z∈[{z_lo:.12g}, {z_hi:.12g}]")

    masks, edges = masks_enu_axis_thirds(pts, axis_idx)
    print(
        f"[2] ENU 축 '{ax_name}' 3등분 경계 (m): "
        f"[{edges[0]:.12g}, {edges[1]:.12g}, {edges[2]:.12g}, {edges[3]:.12g}]"
    )
    for i in range(3):
        n_i = int(masks[i].sum())
        print(f"    구간 {i + 1}: {ax_name}∈[{edges[i]:.12g}, {edges[i + 1]:.12g}]"
              f"{' (상한 포함)' if i == 2 else ''} → 포인트 {n_i}")

    out_base = args.output.expanduser().resolve()
    out_paths = _stem_outputs(out_base, 3)
    txt_base = args.txt_output
    if txt_base is not None:
        txt_base = txt_base.expanduser().resolve()
        ts = txt_base.suffix if txt_base.suffix else ".txt"
        txt_paths = [txt_base.parent / f"{txt_base.stem}_part{k}{ts}" for k in range(1, 4)]
    else:
        txt_paths = [None, None, None]

    prec = max(1, args.txt_precision)
    fmt = f"%.{prec}g"

    for i in range(3):
        mask = masks[i]
        if not np.any(mask):
            print(f"[!] 구간 {i + 1}: 포인트 없음 — PCD 생략", file=sys.stderr)
            continue

        pts_i = pts[mask]
        pc_i = o3d.geometry.PointCloud()
        pc_i.points = o3d.utility.Vector3dVector(pts_i)
        if pc.has_colors():
            c = np.asarray(pc.colors)
            if len(c) == len(pts):
                pc_i.colors = o3d.utility.Vector3dVector(c[mask])

        if args.voxel > 0.0:
            pc_i = pc_i.voxel_down_sample(voxel_size=float(args.voxel))
        pts_ds = np.asarray(pc_i.points, dtype=np.float64)
        print(f"[3-{i + 1}] 구간 {i + 1}: voxel 후 포인트 = {pts_ds.shape[0]}")

        ecef = _pe.enu_points_to_ecef(pts_ds, anchor, r_ecef_enu)
        lo, hi = _pe._xyz_range(ecef)

        out_p = out_paths[i]
        out_p.parent.mkdir(parents=True, exist_ok=True)
        pc_out = o3d.geometry.PointCloud()
        pc_out.points = o3d.utility.Vector3dVector(ecef)
        if pc_i.has_colors() and len(pc_i.colors) == len(pts_ds):
            pc_out.colors = pc_i.colors
        if not o3d.io.write_point_cloud(str(out_p), pc_out, write_ascii=False):
            print(f"PCD 저장 실패: {out_p}", file=sys.stderr)
            sys.exit(1)
        print(f"    저장: {out_p}")
        print(f"    ECEF 범위 X[{lo[0]:.12g},{hi[0]:.12g}] Y[{lo[1]:.12g},{hi[1]:.12g}] Z[{lo[2]:.12g},{hi[2]:.12g}]")

        if txt_paths[i] is not None:
            txt_paths[i].parent.mkdir(parents=True, exist_ok=True)
            np.savetxt(str(txt_paths[i]), ecef, fmt=fmt, delimiter=" ")
            print(f"    txt: {txt_paths[i]}")

    print("[4] 변환식: p_ecef = anchor_ecef_m + R_ecef_enu @ p_enu   (LIGO / gnss_comm)")
    print(f"    R 소스: {r_desc}")
    anch_str = ", ".join(f"{float(x):.18g}" for x in anchor.ravel())
    print(f"    anchor_ecef_m = [{anch_str}]")


if __name__ == "__main__":
    main()
