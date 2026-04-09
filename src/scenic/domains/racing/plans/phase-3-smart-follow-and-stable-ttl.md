# Phase 3: Smart Follow and Stable TTL Choice

## Goal

Introduce conservative tactical behavior that follows safely and repositions without unstable line switching.

## What to Build

Add planner modes:

- `FREE_RUN`
- `FOLLOW`
- `SETUP_LEFT`
- `SETUP_RIGHT`

Behavior rules:

- with no relevant opponent: stay on `optimal`
- when blocked and pass is not safe: `FOLLOW`
- when one side looks promising: `SETUP_LEFT` or `SETUP_RIGHT`
- add hysteresis to prevent rapid left/right bouncing

In this phase, setup is positioning only, not hard pass commitment.

## Why It Matters

This is the first smart-driving layer: ego should avoid blindly driving into slower traffic.

## Success Criteria

In blocked-opponent scenarios:

- ego does not rear-end opponent
- ego follows safely or repositions to a better TTL
- unnecessary TTL switching remains low
- planner stays stable over multiple laps

Suggested targets:

- 0 collisions in follow benchmarks
- minimum gap always above safety threshold
- bounded/non-oscillatory TTL switches per lap
- in free run, most time spent on `optimal`

## Exit Checklist

- [ ] Mode transitions are deterministic and hysteresis-backed.
- [ ] Follow behavior maintains safety gap across benchmarks.
- [ ] Repositioning works without chatter.
- [ ] Multi-lap stability is verified for blocked scenarios.
