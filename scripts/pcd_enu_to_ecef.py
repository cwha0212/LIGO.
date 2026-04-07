#!/usr/bin/env python3
"""
PCD(ENU) 읽기 → 다운샘플링 → ECEF 변환.

LIGO / gnss_comm 과 동일한 정의:
  - p_ecef = anchor_ecef_m + R_ecef_enu @ p_enu   (Eigen 열 벡터와 동일)
  - R_ecef_enu = gnss_comm::ecef2rotation(anchor_ecef)
               = gnss_comm::geo2rotation(gnss_comm::ecef2geo(anchor_ecef))

기본(--rotation gnss_comm): 앵커 ECEF로부터 위와 같이 R을 **재계산**한다.
(YAML에 있는 R_ecef_enu_row_major 는 출력 시 반올림되어 있어 수치가 어긋날 수 있음)

--rotation yaml: YAML에 저장된 R 행렬을 그대로 사용 (디버그용)

Open3D 바이너리 PCD 저장은 좌표가 float32 에 가깝게 양자화된다.
전자리 xyz 텍스트가 필요하면 -o 와 별도로 --txt-output 으로 변환 직후 배열을 저장할 것.

의존성: pip install numpy open3d pyyaml

사용 예:
python3 scripts/pcd_enu_to_ecef.py PCD/scans_3.pcd PCD/scans_3_grid2d.yaml --voxel 0.01 \
-o PCD/scans_3_test.pcd --txt-output PCD/scans_3_test_xyz.txt --enu-z-max 2.5
  # ENU에서 z≥30m 천장 제거 후 변환:
  #   ... --enu-z-max 30
"""

from __future__ import annotations

import argparse
import ast
import re
import sys
from pathlib import Path
from typing import Tuple

import math

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    print("open3d 가 필요합니다: pip install open3d", file=sys.stderr)
    raise SystemExit(1) from e

try:
    import yaml
except ImportError as e:
    print("PyYAML 이 필요합니다: pip install pyyaml", file=sys.stderr)
    raise SystemExit(1) from e


def _read_point_cloud(path: Path) -> o3d.geometry.PointCloud:
    pc = o3d.io.read_point_cloud(str(path))
    if pc.is_empty():
        raise ValueError(f"포인트가 없거나 읽기 실패: {path}")
    return pc


def _xy_range(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """points: (N,3)"""
    xy = points[:, :2]
    return xy.min(axis=0), xy.max(axis=0)


def _xyz_range(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return points.min(axis=0), points.max(axis=0)


# ---------------------------------------------------------------------------
# gnss_comm (HKUST) gnss_utility.cpp 와 동일 상수·함수 — LIGO 가 링크하는 구현
# ---------------------------------------------------------------------------
EARTH_ECCE_2 = 6.69437999014e-3
EARTH_SEMI_MAJOR = 6378137.0


def geo2ecef(lla_deg: np.ndarray) -> np.ndarray:
    """lla: (lat_deg, lon_deg, alt_m). gnss_comm::geo2ecef"""
    lat, lon, h = float(lla_deg[0]), float(lla_deg[1]), float(lla_deg[2])
    cos_lat = math.cos(lat * math.pi / 180.0)
    sin_lat = math.sin(lat * math.pi / 180.0)
    n = EARTH_SEMI_MAJOR / math.sqrt(1.0 - EARTH_ECCE_2 * sin_lat * sin_lat)
    d2r = math.pi / 180.0
    lon_r = lon * d2r
    x = (n + h) * cos_lat * math.cos(lon_r)
    y = (n + h) * cos_lat * math.sin(lon_r)
    z = (n * (1.0 - EARTH_ECCE_2) + h) * sin_lat
    return np.array([x, y, z], dtype=np.float64)


def ecef2geo(xyz: np.ndarray) -> np.ndarray:
    """gnss_comm::ecef2geo — 출력 lla (deg, deg, m)"""
    x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    if x == 0.0 and y == 0.0:
        return np.zeros(3, dtype=np.float64)
    e2 = EARTH_ECCE_2
    a = EARTH_SEMI_MAJOR
    a2 = a * a
    b2 = a2 * (1.0 - e2)
    b = math.sqrt(b2)
    ep2 = (a2 - b2) / b2
    p = math.hypot(x, y)

    s1 = z * a
    s2 = p * b
    h = math.sqrt(s1 * s1 + s2 * s2)
    sin_theta = s1 / h
    cos_theta = s2 / h

    s1 = z + ep2 * b * (sin_theta**3)
    s2 = p - a * e2 * (cos_theta**3)
    h = math.sqrt(s1 * s1 + s2 * s2)
    tan_lat = s1 / s2
    sin_lat = s1 / h
    cos_lat = s2 / h
    lat = math.atan(tan_lat)

    n = a2 * ((a2 * cos_lat * cos_lat + b2 * sin_lat * sin_lat) ** (-0.5))
    alt_m = p / cos_lat - n

    lon = math.atan2(y, x)
    r2d = 180.0 / math.pi
    return np.array([lat * r2d, lon * r2d, alt_m], dtype=np.float64)


def geo2rotation(ref_geo_deg: np.ndarray) -> np.ndarray:
    """gnss_comm::geo2rotation — ENU 벡터를 ECEF 로: v_ecef = R @ v_enu"""
    lat = ref_geo_deg[0] * math.pi / 180.0
    lon = ref_geo_deg[1] * math.pi / 180.0
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    # Eigen::Matrix3d R_ecef_enu << ... (행 순서와 동일)
    return np.array(
        [
            [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
            [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
            [0.0, cos_lat, sin_lat],
        ],
        dtype=np.float64,
    )


def ecef2rotation(ref_ecef: np.ndarray) -> np.ndarray:
    """gnss_comm::ecef2rotation(ref_ecef) == geo2rotation(ecef2geo(ref_ecef))"""
    return geo2rotation(ecef2geo(ref_ecef))


def _parse_triplet_bracket(text: str, key: str) -> np.ndarray:
    """YAML 한 줄에서 key: [a, b, c] 부분을 찾아 numpy 배열로 (문자열 숫자 그대로 eval)."""
    # key 다음부터 줄 끝까지 (주석 제거)
    m = re.search(
        rf"^\s*{re.escape(key)}\s*:\s*\[(.*?)\]\s*(?:#.*)?$",
        text,
        re.MULTILINE | re.DOTALL,
    )
    if not m:
        raise ValueError(f"YAML에서 '{key}:' 트리플렛을 찾지 못했습니다.")
    inner = m.group(1).strip()
    # ast.literal_eval은 공백 구분 float 문자열 리스트에 안전
    vals = ast.literal_eval("[" + inner + "]")
    return np.array([float(x) for x in vals], dtype=np.float64)


def _parse_row_major_r(text: str, key: str) -> np.ndarray:
    m = re.search(
        rf"^\s*{re.escape(key)}\s*:\s*\[(.*?)\]\s*(?:#.*)?$",
        text,
        re.MULTILINE | re.DOTALL,
    )
    if not m:
        raise ValueError(f"YAML에서 '{key}:' 배열을 찾지 못했습니다.")
    inner = m.group(1).strip()
    vals = ast.literal_eval("[" + inner + "]")
    if len(vals) != 9:
        raise ValueError(f"{key} 는 9개 성분이어야 합니다 (got {len(vals)})")
    arr = np.array([float(x) for x in vals], dtype=np.float64)
    return arr.reshape(3, 3)  # row-major 와 C-order reshape 일치


def load_ecef_block_meta(yaml_path: Path) -> dict:
    """
    *_grid2d.yaml 전체 또는 ecef_from_enu 블록만 담은 조각 지원.
    반환: anchor_ecef (필수), r_yaml (YAML에 있으면), lla_deg (있으면)
    """
    raw = yaml_path.read_text(encoding="utf-8")
    if "ecef_from_enu:" not in raw and "anchor_ecef_m:" in raw:
        anchor = _parse_triplet_bracket(raw, "anchor_ecef_m")
        r_yaml = None
        try:
            r_yaml = _parse_row_major_r(raw, "R_ecef_enu_row_major")
        except ValueError:
            pass
        lla = None
        try:
            lla = _parse_triplet_bracket(raw, "anchor_lla_deg_m")
        except ValueError:
            pass
        return {"anchor_ecef": anchor, "r_yaml": r_yaml, "lla_deg": lla}

    data = yaml.safe_load(raw)
    if not isinstance(data, dict):
        raise ValueError("YAML 루트가 mapping 이 아닙니다.")
    block = data.get("ecef_from_enu")
    if block is None:
        raise ValueError("YAML에 ecef_from_enu: 블록이 없습니다.")
    if isinstance(block, str):
        raise ValueError("ecef_from_enu 형식이 올바르지 않습니다 (스칼라 문자열).")

    anchor_list = block.get("anchor_ecef_m")
    if anchor_list is None:
        raise ValueError("ecef_from_enu 안에 anchor_ecef_m 이 필요합니다.")
    anchor = np.asarray(anchor_list, dtype=np.float64).reshape(3)

    r_yaml = None
    r_list = block.get("R_ecef_enu_row_major")
    if r_list is not None:
        r_flat = np.asarray(r_list, dtype=np.float64).reshape(9)
        r_yaml = r_flat.reshape(3, 3)

    lla = None
    lla_list = block.get("anchor_lla_deg_m")
    if lla_list is not None:
        lla = np.asarray(lla_list, dtype=np.float64).reshape(3)

    return {"anchor_ecef": anchor, "r_yaml": r_yaml, "lla_deg": lla}


def enu_points_to_ecef(points_enu: np.ndarray, anchor: np.ndarray, r_ecef_enu: np.ndarray) -> np.ndarray:
    """points_enu: (N,3), p_ecef = anchor + R @ p_enu (Eigen 열 벡터와 동일)."""
    return points_enu @ r_ecef_enu.T + anchor


def resolve_r_ecef_enu(mode: str, meta: dict) -> Tuple[np.ndarray, str]:
    """R 과 설명 문자열."""
    anchor = meta["anchor_ecef"]
    r_yaml = meta["r_yaml"]
    lla = meta["lla_deg"]

    if mode == "yaml":
        if r_yaml is None:
            raise ValueError("--rotation yaml 은 YAML에 R_ecef_enu_row_major 가 필요합니다.")
        return r_yaml, "YAML에 저장된 R_ecef_enu_row_major (출력 반올림본)"

    if mode == "lla":
        if lla is None:
            raise ValueError("--rotation lla 는 anchor_lla_deg_m 이 필요합니다.")
        r = geo2rotation(lla)
        return r, "gnss_comm::geo2rotation(anchor_lla_deg_m)"

    # gnss_comm (기본): laserMapping 저장 시 ecef2rotation(first_gps_ecef)
    r = ecef2rotation(anchor)
    return r, "gnss_comm::ecef2rotation(anchor_ecef_m) == geo2rotation(ecef2geo(anchor_ecef_m))"


def main() -> None:
    ap = argparse.ArgumentParser(description="PCD ENU → ECEF 변환 (grid2d YAML 메타데이터 사용)")
    ap.add_argument("pcd", type=Path, help="입력 PCD (ENU, x/y/z)")
    ap.add_argument(
        "grid_yaml",
        type=Path,
        help="*_grid2d.yaml (ecef_from_enu 포함) 또는 해당 블록만 복사한 .yaml",
    )
    ap.add_argument(
        "--rotation",
        choices=("gnss_comm", "yaml", "lla"),
        default="gnss_comm",
        help="R 계산 방식: gnss_comm(기본)=ecef2rotation(앵커ECEF), yaml=YAML행렬, lla=geo2rotation(LLA)",
    )
    ap.add_argument(
        "--enu-z-max",
        type=float,
        default=None,
        metavar="M",
        help="최초 ENU 좌표계에서 z≥M (m) 인 포인트를 제거한 뒤 다운샘플·ECEF (미지정 시 필터 없음)",
    )
    ap.add_argument(
        "--voxel",
        type=float,
        default=0.5,
        metavar="M",
        help="VoxelLeaf 크기 (m). 0 이하면 다운샘플 생략",
    )
    ap.add_argument("-o", "--output", type=Path, default=None, help="ECEF PCD 저장 경로 (선택, 바이너리는 정밀도 제한 있음)")
    ap.add_argument(
        "--txt-output",
        type=Path,
        default=None,
        metavar="PATH",
        help="변환 직후 float64 xyz를 텍스트로 저장 (PCD 라운드트립 없이 전자리 유지, 권장)",
    )
    ap.add_argument(
        "--txt-precision",
        type=int,
        default=17,
        metavar="N",
        help="--txt-output 의 %%g 유효숫자 (기본 17)",
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

    meta = load_ecef_block_meta(yaml_path)
    anchor = meta["anchor_ecef"]
    r_ecef_enu, r_desc = resolve_r_ecef_enu(args.rotation, meta)

    pc = _read_point_cloud(pcd_path)
    pts = np.asarray(pc.points, dtype=np.float64)
    n0 = pts.shape[0]

    if args.enu_z_max is not None:
        z_th = float(args.enu_z_max)
        mask = pts[:, 2] < z_th
        n_kept = int(mask.sum())
        if n_kept == 0:
            raise ValueError(
                f"ENU z<{z_th} m 조건으로 포인트가 모두 제거되었습니다. (--enu-z-max 조정)"
            )
        pts = pts[mask]
        cols = np.asarray(pc.colors) if pc.has_colors() else None
        if cols is not None and len(cols) == n0:
            cols = cols[mask]
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
    xy_min, xy_max = _xy_range(pts)
    z_lo, z_hi = pts[:, 2].min(), pts[:, 2].max()
    print(f"[1] 입력 PCD: {pcd_path.name}  포인트 수 = {n0}")
    print(f"    x 범위: [{xy_min[0]:.12g}, {xy_max[0]:.12g}]")
    print(f"    y 범위: [{xy_min[1]:.12g}, {xy_max[1]:.12g}]")
    print(f"    z 범위 (ENU): [{z_lo:.12g}, {z_hi:.12g}]")

    if args.voxel > 0.0:
        pc = pc.voxel_down_sample(voxel_size=float(args.voxel))
        pts = np.asarray(pc.points, dtype=np.float64)
        print(f"[2] Voxel 다운샘플 (leaf={args.voxel} m) 후 포인트 수 = {pts.shape[0]}")
    else:
        print("[2] 다운샘플 생략 (--voxel <= 0)")

    ecef = enu_points_to_ecef(pts, anchor, r_ecef_enu)
    lo, hi = _xyz_range(ecef)
    print("[3] 변환: p_ecef = anchor_ecef_m + R_ecef_enu @ p_enu   (LIGO / gnss_comm)")
    print(f"    R 소스: {r_desc}")
    if args.rotation == "gnss_comm" and meta["r_yaml"] is not None:
        d = np.abs(r_ecef_enu - meta["r_yaml"]).max()
        print(f"    (참고) YAML에 적힌 R 과의 최대 성분 차이: {d:.6e}")
    anch_str = ", ".join(f"{float(x):.18g}" for x in anchor.ravel())
    print(f"    anchor_ecef_m = [{anch_str}]")
    print("[4] ECEF 좌표 범위 (다운샘플 후)")
    print(f"    X 범위: [{lo[0]:.12g}, {hi[0]:.12g}]")
    print(f"    Y 범위: [{lo[1]:.12g}, {hi[1]:.12g}]")
    print(f"    Z 범위: [{lo[2]:.12g}, {hi[2]:.12g}]")

    if args.txt_output:
        txt_path = args.txt_output.expanduser().resolve()
        txt_path.parent.mkdir(parents=True, exist_ok=True)
        prec = max(1, args.txt_precision)
        fmt = f"%.{prec}g"
        np.savetxt(str(txt_path), ecef, fmt=fmt, delimiter=" ")
        print(f"    xyz 텍스트(변환 직후 float64): {txt_path}")

    if args.output:
        out = args.output.expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        pc_out = o3d.geometry.PointCloud()
        pc_out.points = o3d.utility.Vector3dVector(ecef)
        if len(pc.colors) == len(pts):
            pc_out.colors = pc.colors
        if not o3d.io.write_point_cloud(str(out), pc_out, write_ascii=False):
            print(f"PCD 저장 실패: {out}", file=sys.stderr)
            sys.exit(1)
        print(f"    저장(PCD 바이너리≈float32 정밀도, 시각화용): {out}")
        if not args.txt_output:
            print(
                "    참고: 이 PCD를 pcd_xyz_to_txt.py 로 읽으면 좌표가 float32 수준으로만 보인다. "
                "전자리 txt는 --txt-output PATH 로 함께 저장할 것.",
            )


if __name__ == "__main__":
    main()

# python3 scripts/pcd_enu_to_ecef.py PCD/scans_3.pcd PCD/scans_3_grid2d.yaml --voxel 0.5 \
#   -o PCD/scans_3_ecef.pcd \
#   --txt-output PCD/scans_3_ecef_0.5.txt