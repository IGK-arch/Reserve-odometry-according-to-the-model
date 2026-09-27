# Independent submission review

Reviewed the pending submission documentation, plot generator and release selection against baseline `4a596c9`, the exact six organizer requirements, current runtime/configuration, and round-4 evidence. No runtime files differ from that baseline. All six requested submission fields are represented. Equations/default values, official versus offline score separation, measured resource numbers, ROS wrapper flags and local/release links were checked. No important or critical finding was identified.

## Actionable minor findings

1. **[P3] Qualify the heading fallback description.** `docs/submission/LIMITATIONS.md:11` says that absence of paired RTK course means map heading is used. That is true for an RTK position anchor with an insufficient RTK companion, but not for every startup: after the deadline, with no RTK fixes, `Navigation::tryStartupAnchor` can call `finalizeStartupAnchor(..., false)`, and a valid ordinary-fix pair can supply `dual_antenna` heading (`navigation.cpp:154–155,276–322`). Suggested wording: “Нет пригодного парного курса для выбранного качества стартовой привязки”, with map heading as the mapped fallback and configured heading when no map exists. This avoids overstating RTK-only heading behavior.

2. **[P3] Document the optional plotting dependency.** `tools/make_submission_plot.py:9–11` imports Matplotlib, but the documented offline prerequisites and `evaluation/requirements.txt` provide only NumPy/SciPy. A reader reproducing the linked graph in an otherwise documented clean environment gets `ModuleNotFoundError: matplotlib`. Add an explicit optional Matplotlib install instruction near graph reproduction, or an appropriately scoped plotting requirements file. It is not needed by the ROS node or the numeric evaluator.

The PNG, metadata JSON and plot script are selected for release; their stored hashes agree. The six GitHub submission links must become available when the parent publishes these pending files. Docker launch/plot execution are independently being verified by the parent; this review does not duplicate those checks. No reviewed source or documentation was edited; only this note was added.

## Resolution

Both minor findings were addressed before packaging: the limitation now distinguishes the selected startup anchor quality and permits ordinary-pair heading for non-RTK fallback; RESULTS.md gives the optional Matplotlib installation command beside plot reproduction.
