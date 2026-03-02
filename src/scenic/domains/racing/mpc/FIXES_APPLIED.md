# Fixes applied (closed-loop and cleanup)

These items were applied in code; the logic lives in the codebase (single source of truth).

- **Closed-loop segment indexing:** Segment indices run 0..n_wp-1; last segment is (wp[n-1] → wp[0]). Applied in `reference_builder.py` (project_to_spline), `mpc_lateral.py` (segment scan, psi_ref_logging), `behaviors.scenic` (heading diff, lookahead CTE, fallback segment).
- **Grade / curvature / cumdist:** Behaviors build segment arrays with wrap (e.g. `(i+1) % nwp`); lookahead uses modulo distance. See behaviors.scenic grade profile and curvature cache.
- **Segment scan and reacquire:** Single helper `_best_segment_in_window` and one reacquire path in `mpc_lateral._compute_errors`; hysteresis skipped after reacquire.

- **CTE single definition:** Safety envelope (throttle/brake/steer limits) uses MPC e_y when available; legacy CTE only as fallback when MPC did not run. Mismatch guard and To-Do B fallback state removed.

Former notes that described these fixes (temp.md, temp2.md, temp3.md) were removed to avoid duplicate, outdated snippets. For current behavior, read the code and docstrings.
