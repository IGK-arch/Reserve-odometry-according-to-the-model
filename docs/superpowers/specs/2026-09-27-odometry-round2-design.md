# Round 2: reliable odometry for 30618

User authorization: continue experiments and improvements to completion, respecting organizer rules. Main scored vehicle is 30618; 30639 remains supported through configuration. No additional approval gate is needed for this authorized implementation.

The frozen comparison version is /private/tmp/odometry-round2-baseline, copied byte-for-byte before changes; /private/tmp/odometry-round2-start-hashes.json records source hashes. Official public bag is diagnostic, already examined, never a parameter-selection target. Preserve input timestamps and causal processing; reference topic must never enter runtime. GNSS-fix corrections are allowed when supplied by organizers. Sensor speed is km/h; output is m/s; position is base_link XYZ.

## Independent investigations

1. Nominal speed: compare existing model extrapolation with wheel-derived short-term acceleration and stronger nominal wheel fusion. Fit/select on train, report whole-session train results, then validation, historical holdout and official reference. Simultaneously test dropout errors. No timestamp shifting or future interpolation. Reject approaches trading broad regressions for one public-bag score.
2. Common wheel jumps: preserve independent evidence that a simultaneous pair was rejected; pair agreement alone must not clear a persistent common jump. Test sustained jump, return to true readings, correct reacquisition after dropout, legitimate acceleration and reset. Do not claim detection of unobservable smooth common bias.
3. GNSS position: evaluate bounded uncertainty-based correction and pair-consistency evidence. Existing systematic lone-antenna bias must be documented and stress-tested, without choosing thresholds from its validation ID. Keep branch switching separate and preserve useful late corrections. GNSS never updates speed or integrated wheel distance.

## Acceptance

Every runtime change needs a meaningful failing behavioral regression followed by a passing fix. Preserve baseline and commands/hashes. Use exact common sample masks; group results by recording session, not millions of independent-looking samples. Session cross-validation cannot be honestly claimed unless learned tables/maps are refit per fold. Include predeclared synthetic faults (single/both drop, jump, frozen, slow bias, delayed GNSS) with known underlying motion, before/after metrics and limitations. Run integrated C++/Python tests, sanitizers and full 1x official ROS check under 2 CPU/512 MiB. Independently inspect final changes and package/rebuild source ZIP. Copy finished work back to main only with unchanged-file hash guards. No push or external submission.
