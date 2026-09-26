"""Check route geometry with perfect GNSS speed, only as an offline diagnostic.

This tests map/initialisation error, not the submitted odometry estimator. It
uses validation and holdout GNSS *after* the route CSV has been built and does
not modify that CSV. Run: py -3.12 tools/map_position_audit.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from map_builder import load_clean_track  # noqa: E402
from route_map_io import RouteMap  # noqa: E402


def summarize_error(err: np.ndarray, s: np.ndarray) -> dict:
    def region(mask: np.ndarray) -> dict:
        if not mask.any():
            return {"n": 0}
        values = err[mask]
        return {"n": int(mask.sum()), "rmse_m": float(np.sqrt(np.mean(values**2))),
                "median_m": float(np.median(values)), "p90_m": float(np.quantile(values, .9))}
    return {
        "first_100m": region(s <= 100),
        "first_500m": region(s <= 500),
        "whole": region(np.ones(len(s), dtype=bool)),
        "end_error_m": float(err[-1]),
    }


def main() -> None:
    route = RouteMap.from_csv(ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets" / "route_map.csv")
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text(encoding="utf-8"))
    results = []
    for split in ("validation", "holdout"):
        for bag in manifest["representatives"][split]:
            try:
                track = load_clean_track(bag)
            except (ValueError, StopIteration) as exc:
                print(f"Skip {bag}: {exc}", flush=True)
                continue
            direction = "out" if track.xyz[0, 0] > -200 else "return" if track.xyz[0, 0] < -4400 else None
            if direction is None:
                print(f"Skip nonterminal {bag}", flush=True)
                continue
            start = route.project(*track.xyz[0, :2], direction=direction)
            map_points = route.rows[direction]
            ref_s = np.asarray(route.abscissae[direction])
            ref_xyz = np.asarray([row[1:] for row in map_points])
            s_mapped = np.clip(start.s + track.s, 0.0, ref_s[-1])
            base = np.column_stack([np.interp(s_mapped, ref_s, ref_xyz[:, j]) for j in range(3)])
            residual = track.xyz[0] - base[0]
            candidate_metrics = {}
            for length in (0, 50, 100, 200):
                correction = 0.0 if length == 0 else np.exp(-track.s / length)[:, None]
                predicted = base + correction * residual
                err = np.linalg.norm(predicted - track.xyz, axis=1)
                candidate_metrics[str(length)] = summarize_error(err, track.s)
            results.append({"split": split, "bag": bag, "direction": direction,
                            "start_projection_distance_m": start.horizontal_distance_m,
                            "startup_residual_xyz_m": residual.tolist(),
                            "metrics_by_decay_length_m": candidate_metrics})
            print(f"{split} {bag} {direction} start={start.horizontal_distance_m:.1f}m "
                  f"100m RMSE raw={candidate_metrics['0']['first_100m']['rmse_m']:.2f} "
                  f"decay100={candidate_metrics['100']['first_100m']['rmse_m']:.2f}", flush=True)
    summary = {}
    for split in ("validation", "holdout"):
        part = [r for r in results if r["split"] == split]
        summary[split] = {"evaluated_bags": len(part)}
        for length in (0, 50, 100, 200):
            for region in ("first_100m", "first_500m", "whole"):
                values = [r["metrics_by_decay_length_m"][str(length)][region]["rmse_m"] for r in part]
                summary[split][f"decay_{length}_{region}_bag_median_rmse_m"] = float(np.median(values)) if values else None
    output = ROOT / "tools" / "map_position_audit.json"
    output.write_text(json.dumps({"summary": summary, "runs": results}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
