# Racing control contract

Single, non-contradictory contract for steering (and throttle/brake) across behavior, MPC, and dSPACE.

## Steering units

- **PID path** (e.g. `FollowRacingLineBehavior`): Steering is **normalized [-1, 1]**. The simulator converts to road wheel radians via `steer_norm * DELTA_MAX_RAD` before writing to hardware.
- **MPC path** (e.g. `FollowRacingLineMPCBehavior`): Steering is **road wheel angle in radians**. The behavior passes the MPC output (rad) through `SetSteerAction`; the simulator writes it after rad→deg conversion in `steer_io` only.

The simulator knows which path is active via `agent._racing_steer_units`, set by `getRacingControllers(agent, use_mpc=...)`:
- `'rad'` → value is road wheel angle (rad); use as-is for ego; for fellows convert to normalized for physics.
- `'normalized'` → value is [-1, 1]; convert to rad with `steer * DELTA_MAX_RAD` before rad→deg.

## Constants

All steering limits and conversion constants live in `scenic.domains.racing.constants`:
- `DELTA_MAX_RAD` — maximum road wheel angle (rad)
- `THETA_SW_MAX_DEG` — dSPACE steering wheel full lock (deg)
- `R` — conversion ratio (used only in `steer_io`)

Do not hardcode 0.2816 or 240 elsewhere.

## dSPACE IO

- **Ego steering:** Value in `_control_state['steering']` is interpreted per `_racing_steer_units`; conversion to steering wheel deg happens **only** in `simulators/dspace/steer_io.py` via `road_rad_to_dspace_value`.
- **Fellow steering:** Physics model expects [-1, 1]. When `_racing_steer_units == 'rad'`, the controller converts rad→norm before calling physics.

## SetSteerAction (driving domain)

`SetSteerAction(steer)` documents steer in **[-1, 1]** for the driving domain. For racing MPC, the behavior passes radians; the value still lies in [-1, 1] in magnitude (since |rad| ≤ 0.28). Simulators that support racing **must** interpret steering using `_racing_steer_units`, not assume [-1, 1] is the only convention.

## SetVehicleControl (dSPACE)

For ego with MPC, `steer` is in **radians**. For normalized use (PID or tests), `steer` is in [-1, 1]; the simulator converts using `DELTA_MAX_RAD`.
