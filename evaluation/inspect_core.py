"""Print the largest estimator errors on one bag for debugging."""

from __future__ import annotations

import argparse

from causal_baseline import causal_wheel_baseline, make_reference, read_run
from core_benchmark import DEFAULT_EXE, reference_at, replay_cpp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--count", type=int, default=20)
    args = parser.parse_args()
    events, truth = read_run(args.bag)
    core = replay_cpp(DEFAULT_EXE, args.bag, events)
    baseline = {x.stamp_ns: x for x in causal_wheel_baseline(events)}
    reference = make_reference(truth)
    t0 = min(stamp for _, stamp, _, _ in events)
    rows = []
    for stamp in baseline.keys() & core.keys():
        ref = reference_at(reference, stamp)
        if ref is None:
            continue
        b, c = baseline[stamp], core[stamp]
        rows.append((abs(c["velocity_mps"] - ref), stamp, ref, b, c))
    rows.sort(reverse=True)
    print("outputs", len(core), "model_only", sum(x["model_only"] for x in core.values()))
    print("worst |core - GNSS|, times from first input stamp")
    for error, stamp, ref, base, core_row in rows[: args.count]:
        print(
            f"t={(stamp-t0)/1e9:8.3f} err={error:6.3f} "
            f"ref={ref:6.3f} base={base.speed_mps:6.3f} "
            f"core={core_row['velocity_mps']:6.3f} "
            f"weights={core_row['front_weight']:.0f}/{core_row['rear_weight']:.0f} "
            f"slip={int(core_row['front_slip'])}/{int(core_row['rear_slip'])}"
        )


if __name__ == "__main__":
    main()
