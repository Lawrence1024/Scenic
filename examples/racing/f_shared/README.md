# Shared F Scenario Bank (`f_shared`)

Canonical regression suite for the smart-ego racing stack. The 17 scenarios below cover ego-alone baseline, single-fellow interactions, deterministic occupied corridors, corner-entry stress, and adversarial blocker behaviors. They are the test set used by the **SD-44 baseline** (F-bank 11/11 collision-free).

The scenario name `F0..F14` is purely an ID — phase numbering was removed in CC-3 (see [`docs/cleanup_inventory.md`](../../../docs/cleanup_inventory.md)).

## Scenarios

| ID | Description |
|---|---|
| `F0` | Ego alone baseline (no fellow) |
| `F1` | Fellow behind on optimal TTL, cruise |
| `F2` | Fellow ahead on optimal TTL, slower cruise |
| `F3L` | Fellow ahead on **left** TTL, cruise |
| `F3R` | Fellow ahead on **right** TTL, cruise |
| `F4` | Fellow ahead then sudden stop |
| `F5` | Fellow ahead then right-left swerve + stop |
| `F6` | Deterministic left-occupied corridor |
| `F7` | Deterministic right-occupied corridor |
| `F8` | Corner-entry, fellow-ahead stress |
| `F9` | Stationary roadside obstacle |
| `F10` | Corner-entry, fellow-left-occupied |
| `F11` | Corner-entry, fellow-right-occupied |
| `F12` | Corner-entry, fellow sudden stop |
| `F13` | Fellow ahead, always faster (unreachable) |
| `F13c` | Control variant: no fellow (sanity check for F13) |
| `F14` | Fellow ahead, active blocker (adversarial) |

## Running

Single scenario, single sample (smoke):

```powershell
scenic examples/racing/f_shared/F1_fellow_behind_optimal_cruise.scenic --2d --simulate --count 1 -b
```

Full F-bank regression (all 17 with the smart-ego stack):

```powershell
python -m scenic.domains.racing.benchmarks.full_stack_runner
```

Subset by name:

```powershell
python -m scenic.domains.racing.benchmarks.full_stack_runner --scenario F4_fellow_ahead_sudden_stop.scenic --repeats 1
python -m scenic.domains.racing.benchmarks.full_stack_runner --scenario-glob "F1[0-4]_*.scenic"
```

## See also

- [`examples/racing/README.md`](../README.md) — racing examples layout + runner index
- [`src/scenic/domains/racing/README.md`](../../../src/scenic/domains/racing/README.md) — full runner registry, control contract, architecture
- [`docs/racing_smart_driving.md`](../../../docs/racing_smart_driving.md) — SD-* smart-ego architecture
