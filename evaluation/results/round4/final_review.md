# Final independent review before publication

**No actionable findings in the reviewed runtime, experiment decisions, metric claims or release selection.** Reviewed runtime `8164061` against immutable `5a82712`, with the current round-4 evidence and Russian summary.

The production Navigation source matches the frozen accepted `pure` source after removing comments and whitespace. The longitudinal estimator source, estimator header and default configuration are byte-identical to baseline; none of the five experimental Kalman candidates is enabled. All current runtime/asset hashes match `final_freeze.json`.

The new report's 12-candidate count, startup/position rejection claims, fifth-candidate pooled metrics, coverage and public comparisons agree with their saved evidence. Independently recomputing the fifth-candidate pooled nominal/dropout results reproduces the displayed values. The public ablation files match the hashes in `kalman_public_provenance.json`; the worse public speed and endpoint error are disclosed. The earlier fourth-candidate generalization table remains explicitly separate from the fifth candidate.

`integration_parity.json` contains 184 matching production/frozen output hashes and references the correct frozen GNSS report hash. The 97-bag longitudinal parity report uses identical baseline/production executable hashes. The common-mask and missing-proxy limitations are retained. Existing verification logs show 19/19 portable CTest and 40/40 Python tests.

The release selector was evaluated without building or modifying the archive: all selected files passed its path, size and forbidden-magic checks. The new production tests, experimental fixture sources, retained fixture CSV/JSON evidence and linked round-4 documents are included. No new raw bag, archive or NumPy data path appears in the tracked delta or unignored pending additions. Reviewed document links resolve and their file targets are included in the release selection.

The official ROS 1× run was still pending at review time and is correctly labelled pending in the report. This review does not certify its eventual score, nor claim that the final source ZIP has already been regenerated. Those are the remaining planned publication steps, not defects in the reviewed code or evidence. No runtime or documentation changes were made by this review; only this review note was added.
