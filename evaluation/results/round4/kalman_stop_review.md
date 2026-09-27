# Independent re-review: frozen long_guarded_stop

**The structural standstill repair resolves both review blockers in the tested
cases. It does not justify changing the default estimator.** The legacy default
is retained under the broader selection decision; this review examines the repair
and does not relabel previously examined public/held-out runs as independent data.

Compared actual frozen `long_guarded_stop/estimator.cpp` with `long_guarded`:
the only functional additions clear `kalman_residual_mps2_` and `kalman_va_`
inside the existing authoritative dual-wheel standstill clamp. The narrow earlier
reset remains, but cannot bypass the new final reset. Candidate and production
sources were not edited.

## Independent fixtures

Source and runner are retained in `review_fixtures/kalman_stop_probe.cpp` and
`review_fixtures/run_kalman_stop_review.py`, with full CSV and JSON evidence.
The same fixture is compiled against immutable baseline, original guarded KF,
and the repaired fifth variant. It inspects the actual private covariance in a
read-only diagnostic build, without changing the implementation.

The 24 cases cover:

- Both vehicle configurations, including30639's default wheel sigma0.085.
- 5,10 and50Hz wheel/controller input.
- Wheel standstill readings0.04 and0.054m/s, both inside the existing0.055 band.
- Initial standstill and genuine6m/s motion followed by braking to zero, then
  two seconds of stopped observations before loss.
- Ten seconds with no wheel updates and continued neutral controller input.
- State/covariance inspection after every controller/front/rear event and an
  explicit estimator reset afterward.

Vehicle-specific sensor scales are inverted when constructing raw wheel input,
so the physical measured speed is exactly the intended value. The braking case
uses a continuous1m/s² speed decrease and a braking command, followed by neutral
at the stop. Existing small distance increments before wheel loss are not
misreported as blackout motion; the fixture saves distance at the loss boundary.

| Outcome over24 cases | Baseline | Original guarded KF | Fifth variant |
| --- | ---: | ---: | ---: |
| Maximum speed during stopped dropout, m/s | 0 | 6.477758 | 0 |
| Maximum added distance during dropout, m | 0 | 48.724149 | 0 |
| Negative covariance observations | N/A | 8 | 0 |
| Nonfinite covariance observations | N/A | 0 | 0 |
| Maximum stopped acceleration residual, m/s² | N/A | 2.353594 | 0 |

The fifth variant's minimum observed covariance determinant was
**4.621342419736444e−5**. Every case reset residual and cross-covariance to zero.
The original variant's worst case was30618 at50Hz with0.054m/s stopped noise
following braking; the expanded sweep explains why its error is larger than the
first0.04m/s10Hz counterexample. All newly asserted fifth-variant checks passed.
These results close the two specific standstill findings, not every possible
numerical or statistical issue in the observer.

## Long-dropout tail interpretation

The residual acceleration itself is **not constant forever**: the actual code
uses an Ornstein–Uhlenbeck mean transition with3s decay time. With no new wheel
measurement, an initial residual `a0` contributes `a0·exp(−t/3)` to acceleration.
Its integral can nevertheless leave a lasting velocity offset.

For a local comparison with identical physical-model acceleration, ignoring speed
clamps and nonlinear feedback, the residual contribution is

```
Δv(T) = a0·τ·(1 − exp(−T/τ))
Δs(T) = a0·τ·[T − τ·(1 − exp(−T/τ))],  τ=3s.
```

For example,a0=0.5m/s² andT=10s imply1.4465m/s added velocity and10.6605m added
position under those stated assumptions. These are conditional model equations,
not a measured error on a real recording. Once acceleration has mostly decayed,
the accumulated velocity difference can still make position divergence grow;
nonlinear drag, braking or a zero-speed clamp can subsequently alter it.

Retaining residual may help when it represents a still-valid model discrepancy,
such as an external force compensating modeled drag or sustained acceleration
missing from the physical prior. If that discrepancy changes or disappears during
wheel loss, the old residual can instead add erroneous acceleration. Driver
command changes still update the physical model; the current implementation does
not immediately clear the residual on every command change. A positive residual
can temporarily offset modeled braking, while a negative one can oppose modeled
traction. This is a conditional persistence assumption, not evidence that every
command transition is mishandled. External-force changes are not directly
observable from the missing wheels.

The repaired standstill case is different: the algorithm already possesses
explicit stationary evidence and now enforces it consistently. The broader
public-velocity and held-out-tail selection concerns are not removed by this fix;
there is no basis here for promoting the KF to the default.

## Reproduction

```sh
python3 evaluation/results/round4/review_fixtures/run_kalman_stop_review.py
```

The runner compiles only frozen sources into `/private/tmp/round4-kalman-review`.
Source, header and fixture-binary SHA-256 values are in
`review_fixtures/kalman_stop_results.json`. The retained fifth source hash is
`a2788799d272f4d4c30af7c8f1e6080cc5cf6132026e5ab99c307f57079d1ecd`.
