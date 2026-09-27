#!/usr/bin/env python3
"""Compare complete causal C++ estimator outputs on every unique supplied bag.

This is a regression gate, not a score: missing GNSS does not hide changed
outputs. Both executables receive exactly the same permitted C/F/R events.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from causal_baseline import CMD, FRONT, REAR, read_run

ROOT = Path(__file__).resolve().parents[1]
CODES = {CMD: "C", FRONT: "F", REAR: "R"}


def replay(exe: Path, bag: str, payload: str, table: Path | None) -> bytes:
    args = [str(exe.resolve()), bag.split("_", 1)[0]]
    if table is not None and bag.startswith("30618_"):
        args.append(str(table.resolve()))
    result = subprocess.run(args, input=payload.encode(), stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, cwd=ROOT, timeout=180,
                            check=True)
    return result.stdout


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--table", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    manifest = json.loads((ROOT / "tools/split_manifest.json").read_text())
    rows = []
    for split in ("train", "validation", "holdout"):
        for bag in manifest["representatives"][split]:
            events, _ = read_run(bag)
            payload = "".join(
                f"{receive},{stamp},{CODES[topic]},{value:.17g}\n"
                for receive, stamp, topic, value in events
            )
            before = replay(args.before, bag, payload, args.table)
            after = replay(args.after, bag, payload, args.table)
            rows.append({"split": split, "bag": bag, "inputs": len(events),
                         "outputs_before": before.count(b"\n") - 1,
                         "outputs_after": after.count(b"\n") - 1,
                         "identical": before == after,
                         "before_sha256": hashlib.sha256(before).hexdigest(),
                         "after_sha256": hashlib.sha256(after).hexdigest()})
            print(split, bag, "same" if before == after else "CHANGED", flush=True)
    report = {"protocol": "Same receive-order C/F/R bytes; complete CSV stdout compared byte for byte.",
              "before_exe_sha256": hashlib.sha256(args.before.read_bytes()).hexdigest(),
              "after_exe_sha256": hashlib.sha256(args.after.read_bytes()).hexdigest(),
              "bags": len(rows), "identical_bags": sum(r["identical"] for r in rows),
              "inputs": sum(r["inputs"] for r in rows),
              "outputs_before": sum(r["outputs_before"] for r in rows),
              "outputs_after": sum(r["outputs_after"] for r in rows),
              "rows": rows}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("saved", args.out)


if __name__ == "__main__":
    main()
