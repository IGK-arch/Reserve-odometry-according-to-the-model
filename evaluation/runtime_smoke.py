"""Measure standalone estimator replay throughput on one real bag.

This includes CSV stdin/stdout for the C++ process, but excludes ROS transport
and bag playback. It is a compute margin check, not input-to-publication latency.
"""

from __future__ import annotations

import argparse
import statistics
import subprocess
import time
from pathlib import Path

from causal_baseline import CMD, FRONT, REAR, read_run
from core_benchmark import DEFAULT_EXE, compile_cli


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", nargs="?", default="30618_2050d396")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--drive-table", type=Path,
                        help="Use the train-only table as in the 30618 ROS default")
    args = parser.parse_args()
    if args.build:
        compile_cli(DEFAULT_EXE)
    events, _ = read_run(args.bag)
    codes = {FRONT: "F", REAR: "R", CMD: "C"}
    payload = "".join(f"{recv},{stamp},{codes[topic]},{value}\n"
                      for recv, stamp, topic, value in events)
    t_min = min(stamp for _, stamp, _, _ in events)
    t_max = max(stamp for _, stamp, _, _ in events)
    bag_seconds = (t_max - t_min) * 1e-9
    trials = []
    outputs = None
    command = [str(DEFAULT_EXE.resolve()), args.bag.split("_", 1)[0]]
    if args.drive_table:
        command.append(str(args.drive_table.resolve()))
    for _ in range(args.repeats):
        start = time.perf_counter()
        result = subprocess.run(
            command,
            input=payload, text=True, capture_output=True, check=True,
        )
        trials.append(time.perf_counter() - start)
        outputs = result.stdout.count("\n") - 1
    median_s = statistics.median(trials)
    print(f"bag={args.bag} events={len(events)} outputs={outputs} "
          f"bag_duration_s={bag_seconds:.1f}")
    print(f"median_replay_wall_s={median_s:.3f} "
          f"events_per_s={len(events) / median_s:.0f} "
          f"realtime_margin={bag_seconds / median_s:.0f}x")
    print("Scope: standalone C++ core plus CSV pipes; ROS callback latency unmeasured.")


if __name__ == "__main__":
    main()
