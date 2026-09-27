"""Export the organiser's height profile without reading any recorded trip.

Usage: python3 tools/build_official_elevation.py /path/to/official-pathgraph.json
XY is retained only to locate the surveyed base_link height; it must not replace
both rails or the terminal loops in the train-derived route geometry.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).resolve().parents[1]
        / "ros2_ws/src/tram_odometry/assets/official_elevation.csv",
    )
    args = parser.parse_args()
    source_bytes = args.source.read_bytes()
    source = json.loads(source_bytes)
    points = [tuple(float(point[key]) for key in ("x", "y", "z"))
              for point in source["points"]]
    if len(points) < 2 or not all(math.isfinite(v) for point in points for v in point):
        raise ValueError("At least two finite XYZ points are required")
    if any(math.hypot(a[0] - b[0], a[1] - b[1]) <= 1e-8
           for a, b in zip(points, points[1:])):
        raise ValueError("Consecutive profile points must have distinct XY")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "x,y,z\n" + "".join(",".join(f"{v:.9f}" for v in point) + "\n" for point in points),
        encoding="utf-8",
    )
    metadata = {
        "source": args.source.name,
        "source_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "csv_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "point_count": len(points),
        "coordinate_frame": "MGRS 37UCB fixed numeric square; x=UTM37N-300000, y=UTM37N-6100000",
        "height_target": "base_link at rail contact elevation; already uses the official z datum",
        "usage": "Height only at final base_link XY; full weight within 5m, fade to zero at 10m",
        "limitations": [
            "Surveyed XY follows the return rail; outgoing rail has a lateral offset.",
            "Profile omits terminal loops; retain train-derived map height outside coverage.",
            "No validation, holdout or checker reference positions were used in this export.",
        ],
    }
    args.output.with_name(args.output.stem + "_meta.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8",
    )
    print(f"Exported {len(points)} official height points to {args.output}")


if __name__ == "__main__":
    main()
