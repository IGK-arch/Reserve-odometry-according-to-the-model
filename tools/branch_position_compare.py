"""Compare frozen A/B route assets with the same causal C++ speed replay.

Validation GNSS is used only as offline truth here.  No map points or
estimator parameters are changed based on this score.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "evaluation"))
from position_proxy import score_bag  # noqa: E402
from core_benchmark import DEFAULT_EXE  # noqa: E402
sys.path.insert(0, str(ROOT / "tools"))
from route_map_io import RouteMap  # noqa: E402


def main() -> None:
    manifest = json.loads((ROOT / "tools" / "split_manifest.json").read_text())
    summaries = {item["bag"]: item for item in json.loads(
        (ROOT / "analysis" / "routes" / "summaries.json").read_text())}
    assets = ROOT / "ros2_ws" / "src" / "tram_odometry" / "assets"
    maps = {branch: RouteMap.from_csv(assets / path) for branch, path in
            (("B", "route_map.csv"), ("A", "route_map_branch_a.csv"))}
    rows = []
    for bag in manifest["representatives"]["validation"]:
        path = summaries[bag]["master_path"]
        if path.get("net_m", 0) < 4400 or path["end_enu"][0] > path["start_enu"][0]:
            continue
        for branch in ("B", "A"):
            result = score_bag(bag, maps[branch], DEFAULT_EXE.resolve())
            if result is None:
                continue
            record = {"branch": branch, **result}
            rows.append(record)
            print(bag, branch, f"base_RMSE={record['base_rmse_3d_m']:.2f}m",
                  f"base_end={record['base_end_error_3d_m']:.2f}m", flush=True)
    output = ROOT / "tools" / "branch_position_compare_validation.json"
    output.write_text(json.dumps(rows, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
