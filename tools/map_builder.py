"""Build a compact 3D railway map from training-session GNSS only.

Run: py -3.12 tools/map_builder.py

The output describes the trajectory of the *master GNSS antenna*, in a fixed
WGS84 ENU frame. All GNSS use stops here: the ROS estimator reads only the
resulting CSV after the permitted initial position fix, if one exists.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d, median_filter
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis" / "routes"))
from analyze_gnss import DATA, enu, extract, finite_positions  # noqa: E402
from route_map_io import (ORIGIN_ALT_M, ORIGIN_LAT_DEG, ORIGIN_LON_DEG,
                          RouteMap, geodetic_to_enu)  # noqa: E402


MANIFEST = ROOT / "tools" / "split_manifest.json"
ASSETS = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets"
CSV_PATH = ASSETS / "route_map.csv"
META_PATH = ASSETS / "route_map_meta.json"
BRANCH_A_CSV_PATH = ASSETS / "route_map_branch_a.csv"
BRANCH_A_META_PATH = ASSETS / "route_map_branch_a_meta.json"
GRID_STEP_M = 2.0

# Chosen by training-only quality audit: complete runs, few fix jumps and
# master/rover separation faults. The first entry anchors longitudinal s.
SOURCES = {
    # The western approach has two ~60 m-separated branches.  Training runs
    # favour B; at least four of 19 long outbound runs reach A, while some
    # stop before the fork.  Anchor on a clean complete B run.  A-branch fixes
    # are kept out of the fusion rather than averaged into a nonexistent rail.
    "out": ["30618_76e1f9c7", "30618_095a115b", "30618_b83d854d", "30618_8158f0b0"],
    "return": ["30618_68847170", "30618_22c1c589", "30618_b8044aa0", "30618_e3d94878"],
}
# Optional west-berth A map.  Selection requires external route knowledge;
# wheel speeds and controller position cannot reveal which rail switch was set.
BRANCH_A_OUT_SOURCES = ["30618_8158f0b0", "30618_49fe4c54", "30618_76e1f9c7", "30618_b83d854d"]


@dataclass
class Track:
    bag: str
    s: np.ndarray
    xyz: np.ndarray
    raw_fix_count: int
    clean_fix_count: int
    velocity_integral_m: float


def _nearest_indices(source_time: np.ndarray, query_time: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    indices = np.searchsorted(source_time, query_time)
    indices = np.clip(indices, 1, len(source_time) - 1)
    use_prev = np.abs(source_time[indices - 1] - query_time) < np.abs(source_time[indices] - query_time)
    indices = indices - use_prev.astype(int)
    return indices, np.abs(source_time[indices] - query_time)


def _median_spatial_bins(s: np.ndarray, xyz: np.ndarray, width: float = 0.5) -> tuple[np.ndarray, np.ndarray]:
    indices = np.floor((s - s[0]) / width).astype(np.int64)
    _, starts = np.unique(indices, return_index=True)
    ends = np.r_[starts[1:], len(indices)]
    s_out = np.array([np.median(s[a:b]) for a, b in zip(starts, ends)])
    p_out = np.array([np.median(xyz[a:b], axis=0) for a, b in zip(starts, ends)])
    return s_out, p_out


def load_clean_track(bag: str) -> Track:
    path = next((DATA / bag).glob("*.db3"))
    _, _, records = extract(path)
    master = finite_positions(records["mf"])
    rover = finite_positions(records["rf"])
    vel = records["mv"]
    if len(master) < 3000 or len(rover) < 3000 or len(vel) < 3000:
        raise ValueError(f"Not enough GNSS in map source {bag}")

    # Work offline in sensor-time order; this sorting never enters the causal
    # estimator. Track geometry is accepted only where two antennas show their
    # physical ~12.44 m separation and master fixes are not isolated jumps.
    master = master[np.argsort(master[:, 1], kind="stable")]
    rover = rover[np.argsort(rover[:, 1], kind="stable")]
    vel = vel[np.argsort(vel[:, 1], kind="stable")]
    pm = enu(master[:, 2], master[:, 3], master[:, 4])
    pr = enu(rover[:, 2], rover[:, 3], rover[:, 4])
    rv_idx, rv_age = _nearest_indices(rover[:, 1], master[:, 1])
    separation = np.linalg.norm(pm[:, :2] - pr[rv_idx, :2], axis=1)
    pair_good = (rv_age <= 0.15) & (separation >= 9.0) & (separation <= 16.0)
    med = median_filter(pm, size=(9, 1), mode="nearest")
    jump_good = np.linalg.norm(pm - med, axis=1) < 3.0
    valid = pair_good & jump_good & (master[:, 5] >= 0)

    vt = vel[:, 1]
    vv = np.linalg.norm(vel[:, 2:5], axis=1)
    valid_velocity = np.isfinite(vt + vv) & (vv <= 20.0)
    vt, vv = vt[valid_velocity], vv[valid_velocity]
    unique_time = np.r_[True, np.diff(vt) > 0.0001]
    vt, vv = vt[unique_time], vv[unique_time]
    if len(vt) < 3000:
        raise ValueError(f"Too little valid GNSS velocity in {bag}")
    vv = np.where(vv < 0.10, 0.0, vv)
    dt = np.diff(vt)
    if np.max(dt) > 5.0:
        raise ValueError(f"Large GNSS velocity gap in {bag}")
    # Trapezoidal speed integration supplies a metric arc coordinate while
    # stationary position jitter cannot spuriously lengthen the route.
    sv = np.r_[0.0, np.cumsum((vv[1:] + vv[:-1]) * 0.5 * dt)]
    midx, mage = _nearest_indices(vt, master[:, 1])
    valid &= mage <= 0.2
    valid &= np.isfinite(pm).all(axis=1)
    valid &= (pm[:, 0] > -5100) & (pm[:, 0] < 200) & (pm[:, 1] > -2000) & (pm[:, 1] < 250)
    if valid.sum() < 3000:
        raise ValueError(f"Too few two-antenna-consistent fixes in {bag}: {valid.sum()}")
    t = master[valid, 1]
    p = pm[valid]
    s = np.interp(t, vt, sv)
    s -= s[0]
    s, p = _median_spatial_bins(s, p)
    if len(s) < 3000 or s[-1] < 4800:
        raise ValueError(f"Map source {bag} does not cover a full trip: {s[-1]:.0f} m")
    grid = np.arange(0.0, math.floor(s[-1] / GRID_STEP_M) * GRID_STEP_M + 0.01, GRID_STEP_M)
    xyz = np.column_stack([np.interp(grid, s, p[:, axis]) for axis in range(3)])
    xyz = gaussian_filter1d(xyz, sigma=1.2, axis=0, mode="nearest")
    return Track(bag, grid, xyz, len(master), int(valid.sum()), float(s[-1]))


def _project_to_reference(ref: Track, other: Track) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project a second same-direction run onto a spatial reference polyline."""
    tree = cKDTree(ref.xyz[:, :2])
    _, near_nodes = tree.query(other.xyz[:, :2], k=2)
    projected_s = np.full(len(other.s), np.nan)
    distance = np.full(len(other.s), np.inf)
    alignment = np.full(len(other.s), -1.0)
    for i, near in enumerate(near_nodes):
        candidates = {int(j) for node in near for j in (node - 1, node) if 0 <= j < len(ref.s) - 1}
        own_i = min(max(i, 1), len(other.xyz) - 2)
        own_tangent = other.xyz[own_i + 1, :2] - other.xyz[own_i - 1, :2]
        own_norm = np.linalg.norm(own_tangent)
        for j in candidates:
            a, b = ref.xyz[j, :2], ref.xyz[j + 1, :2]
            delta = b - a
            norm_sq = np.dot(delta, delta)
            if norm_sq < 1e-6:
                continue
            fraction = np.clip(np.dot(other.xyz[i, :2] - a, delta) / norm_sq, 0, 1)
            point = a + fraction * delta
            d = np.linalg.norm(other.xyz[i, :2] - point)
            if d < distance[i]:
                distance[i] = d
                projected_s[i] = ref.s[j] + fraction * (ref.s[j + 1] - ref.s[j])
                alignment[i] = np.dot(own_tangent, delta) / (own_norm * math.sqrt(norm_sq)) if own_norm > 0.01 else 0.0
    return projected_s, distance, alignment


def _other_on_ref(ref: Track, other: Track) -> tuple[np.ndarray, dict]:
    projected_s, distance, alignment = _project_to_reference(ref, other)
    core = (distance < 4.0) & (alignment > 0.6) & (projected_s > 100) & (projected_s < ref.s[-1] - 100)
    if core.sum() < 500:
        raise ValueError(f"Cannot align {other.bag} with reference {ref.bag}")
    shift = float(np.median(projected_s[core] - other.s[core]))
    good = (distance < 5.0) & (alignment > 0.5) & (np.abs(projected_s - other.s - shift) < 25.0)
    # Ignore small local reversals caused by GNSS noise and avoid interpolating
    # through long rejected sections.
    ps = projected_s[good]
    coords = other.xyz[good]
    order = np.argsort(ps, kind="stable")
    ps, coords = ps[order], coords[order]
    keep = np.r_[True, np.diff(ps) > 0.25]
    ps, coords = ps[keep], coords[keep]
    out = np.column_stack([np.interp(ref.s, ps, coords[:, axis], left=np.nan, right=np.nan)
                           for axis in range(3)])
    indices = np.searchsorted(ps, ref.s)
    left = np.clip(indices - 1, 0, len(ps) - 1)
    right = np.clip(indices, 0, len(ps) - 1)
    out[ps[right] - ps[left] > 12.0] = np.nan
    return out, {
        "bag": other.bag,
        "aligned_points": int(good.sum()),
        "median_horizontal_difference_m": float(np.median(distance[good])),
        "longitudinal_shift_m": shift,
        "contributed_map_fraction": float(np.isfinite(out[:, 0]).mean()),
    }


def build_direction(direction: str, source_bags: list[str] | None = None) -> tuple[Track, list[dict]]:
    tracks = [load_clean_track(bag) for bag in (source_bags if source_bags is not None else SOURCES[direction])]
    ref = tracks[0]
    layers = [ref.xyz]
    diagnostics = [{
        "bag": ref.bag,
        "role": "reference",
        "raw_fix_count": ref.raw_fix_count,
        "clean_fix_count": ref.clean_fix_count,
        "velocity_integral_m": ref.velocity_integral_m,
        "contributed_map_fraction": 1.0,
    }]
    for other in tracks[1:]:
        aligned, report = _other_on_ref(ref, other)
        report.update(role="support", raw_fix_count=other.raw_fix_count,
                      clean_fix_count=other.clean_fix_count,
                      velocity_integral_m=other.velocity_integral_m)
        layers.append(aligned)
        diagnostics.append(report)
    fused = np.nanmedian(np.stack(layers), axis=0)
    fused = gaussian_filter1d(fused, sigma=0.65, axis=0, mode="nearest")
    # The GNSS speed integral is a robust correspondence coordinate during
    # fusion, but a few local velocity dropouts produce >2 m of displacement
    # between nominal 2 m nodes. Reparameterise the finished 3D polyline by
    # its own arc length so that online integration of metres/s stays metric.
    geometric_s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(fused, axis=0), axis=1))]
    metric_s = np.arange(0.0, math.floor(geometric_s[-1] / GRID_STEP_M) * GRID_STEP_M + 0.01, GRID_STEP_M)
    metric_xyz = np.column_stack([np.interp(metric_s, geometric_s, fused[:, axis]) for axis in range(3)])
    return Track("fused:" + direction, metric_s, metric_xyz, 0, 0, ref.velocity_integral_m), diagnostics


def write_csv(path: Path, tracks: dict[str, Track]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["direction", "s", "x", "y", "z"])
        for direction in ("out", "return"):
            track = tracks[direction]
            for s, (x, y, z) in zip(track.s, track.xyz):
                writer.writerow([direction, f"{s:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"])


def main() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    # A source bag may be a nonrepresentative duplicate, so match by group.
    train_members = {bag for group in manifest["groups"] if group["split"] == "train" for bag in group["bags"]}
    source_bags = [bag for bags in SOURCES.values() for bag in bags]
    if not set(source_bags).issubset(train_members):
        raise ValueError("Map source falls outside the frozen training partition")
    if not set(BRANCH_A_OUT_SOURCES).issubset(train_members):
        raise ValueError("Branch A map source falls outside the frozen training partition")
    if len(source_bags) != len(set(source_bags)):
        raise ValueError("Duplicate map source")

    ASSETS.mkdir(parents=True, exist_ok=True)
    directions = {}
    diagnostics = {}
    for direction in ("out", "return"):
        print(f"Building {direction} from {len(SOURCES[direction])} training bags", flush=True)
        directions[direction], diagnostics[direction] = build_direction(direction)
    write_csv(CSV_PATH, directions)
    metadata = {
        "schema_version": 1,
        "csv": CSV_PATH.name,
        "west_berth_branch": "B",
        "target_point": "master GNSS antenna trajectory; judge target is base_link, master TF x=-9.873m z=+3m relative to base_link",
        "coordinate_frame": "WGS84 geodetic EPSG:4979 to ECEF EPSG:4978 to local ENU",
        "origin_lon_deg": ORIGIN_LON_DEG,
        "origin_lat_deg": ORIGIN_LAT_DEG,
        "origin_alt_m": ORIGIN_ALT_M,
        "directions": {direction: {
            "meaning": "east_to_west" if direction == "out" else "west_to_east",
            "s_start": "east terminal" if direction == "out" else "west terminal",
            "length_m": float(track.s[-1]),
            "point_count": len(track.s),
            "sources": diagnostics[direction],
        } for direction, track in directions.items()},
        "source_split_manifest": "tools/split_manifest.json",
        "on_vehicle_gnss_usage": "initial position fix only, never tracking after initialization",
        "limitations": [
            "Default outbound map follows west berth B. Optional branch A is a separate CSV selected only by external route assignment; depot paths are not mapped.",
            "Judge output frame is fixed MGRS-CB numeric coordinates (UTM37N easting-300000, northing-6100000); transform master route to base_link using rigid 3D tangent and specified TF.",
            "GNSS master altitude is kept in internal ENU; GNSS vertical datum is not independently documented.",
        ],
    }
    META_PATH.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Building optional outbound branch A from {len(BRANCH_A_OUT_SOURCES)} training bags", flush=True)
    a_out, a_diag = build_direction("out", BRANCH_A_OUT_SOURCES)
    a_directions = {"out": a_out, "return": directions["return"]}
    write_csv(BRANCH_A_CSV_PATH, a_directions)
    a_metadata = json.loads(json.dumps(metadata))
    a_metadata["csv"] = BRANCH_A_CSV_PATH.name
    a_metadata["west_berth_branch"] = "A"
    a_metadata["directions"]["out"].update(length_m=float(a_out.s[-1]),
                                             point_count=len(a_out.s), sources=a_diag)
    a_metadata["limitations"][0] = "Branch A west berth is mapped for outbound trips; switching the configured map requires an external branch assignment."
    BRANCH_A_META_PATH.write_text(json.dumps(a_metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    route = RouteMap.from_csv(CSV_PATH)
    for direction in ("out", "return"):
        sample = route.sample(direction, directions[direction].s[-1] / 2)
        assert np.isfinite(sample).all()
    enu_origin = geodetic_to_enu(ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_ALT_M)
    assert max(map(abs, enu_origin)) < 1e-5
    print(json.dumps({d: {"length_m": metadata["directions"][d]["length_m"],
                          "point_count": metadata["directions"][d]["point_count"]}
                      for d in ("out", "return")}, indent=2))
    print(CSV_PATH)
    print(META_PATH)
    print(BRANCH_A_CSV_PATH)
    print(BRANCH_A_META_PATH)


if __name__ == "__main__":
    main()
