# Level 2 Swarm Scaling Plan Without Inter-Drone Communication

## Summary

This document defines how the single-drone predictive intercept stack should scale to multiple
defensive drones when the swarm has no runtime communication between agents. Each drone runs the
same software, but with its own `drone_id` and a small amount of preloaded identity/configuration.

Because there is no communication and no preloaded defended boundary, this plan does not target
globally optimal assignment. Instead, it targets:

- deterministic launch-origin-based coverage
- fast local response to detected targets
- minimal clustering of friendlies
- safe local handling of friendly tracks

## Problem Framing

Without communication, the swarm cannot exchange:

- target ownership
- intercept intent
- pose or health
- dynamic sector reallocation

Without a preloaded perimeter, the swarm also cannot divide a known area into fixed sectors ahead
of time. Coverage therefore has to emerge from a local patrol policy centered on launch origin.

## Design Principles

- Use deterministic symmetry breaking so drones do not all choose the same patrol behavior.
- Keep target claiming local and reproducible from on-board observations plus configuration.
- Avoid plans that require consensus, auctions, or live peer-state exchange.
- Scaffold friendly recognition locally using visual markers now, with LED strip signaling added
  later.

## Swarm Policy

### Shared mission assumptions

Each drone is preloaded with:

- `drone_id`
- `drone_role_index`
- `swarm_size_hint`
- launch-origin recording enabled at arming

Each drone independently records its own launch origin and uses that as the center of the defended
ring for patrol and engagement geometry.

### Patrol geometry

- On startup, each drone records:
  - launch position
  - launch heading
- Each drone computes a patrol wedge from:
  - `drone_role_index`
  - `swarm_size_hint`
- Each wedge is defined as an angular slice around launch origin.
- Each drone flies a patrol orbit or loiter anchor inside its wedge.
- If the swarm is deployed in a new area, the wedges still exist because they are generated from
  shared configuration, not from map knowledge.

### Patrol state machine

- `SPREAD`
  - leave launch cluster
  - move to the wedge anchor or initial ring point
- `PATROL`
  - remain inside wedge
  - maintain ring radius around launch origin
- `ENGAGE`
  - intercept locally claimable unknown targets
- `RECOVER`
  - return toward wedge patrol area after intercept or target loss
- `FAILSAFE_HOLD`
  - hold or land on health failures

## Local Target Handling

### Target classes

Each locally perceived track should be classified as:

- `UNKNOWN`
- `FRIENDLY`
- `AMBIGUOUS`

### Friendly detection

- Current scaffold: visual markers only
- Future upgrade: LED strip signaling with known temporal patterns
- The classification path should be designed now so LED-based friendly detection can be added
  later without changing the higher-level swarm logic.

### Engagement rules

- Only `UNKNOWN` targets are eligible for normal interception.
- `FRIENDLY` tracks are excluded from engagement and treated as dynamic keep-out obstacles.
- `AMBIGUOUS` tracks are not engaged unless they enter a breach zone near launch origin.

### Claiming a target without communication

Each drone uses local rules to decide whether to engage:

- the target bearing from launch origin lies inside the drone's wedge
- the predicted intercept cost is below threshold
- the target is not classified as `FRIENDLY`
- local obstacle and safety gates are clear

Tie-break behavior must be deterministic to reduce duplicate engagement:

- primary: target lies in wedge
- secondary: lower local time-to-intercept
- tertiary: lower `drone_role_index`

This does not eliminate duplicate claims completely, but it reduces clustering without requiring
message passing.

### Breach override

If an unknown or ambiguous target enters a configured `breach_radius_m` around launch origin:

- any drone may break wedge discipline
- the nearest intercept-capable drone engages
- other drones continue local avoidance and perimeter recovery behavior

## Required Interface and Message Changes

### Track classification and intercept data

Extend the predicted-track interface to include:

- `classification`
- `classification_confidence`
- `bearing_from_launch_origin_rad`
- `range_from_launch_origin_m`
- `time_to_intercept_s`
- `claimable_by_local_policy`

### Swarm configuration parameters

Add shared configuration keys:

- `drone_role_index`
- `swarm_size_hint`
- `coverage_ring_radius_m`
- `coverage_ring_expand_rate_mps`
- `coverage_wedge_half_angle_deg`
- `patrol_speed_mps`
- `recover_speed_mps`
- `breach_radius_m`
- `claim_hysteresis_s`
- `friendly_keepout_m`

## Implementation Steps

### 1. Build on the single-drone intercept stack

- Finish the one-drone predictor and APN intercept path first.
- Reuse the same `PredictedTrack` contract in every drone instance.

### 2. Add launch-origin frame support

- Record launch pose on arming or mission start.
- Compute each target's bearing and range relative to launch origin.
- Add wedge membership checks and patrol anchor generation.

### 3. Add local patrol controller

- Patrol controller should run when no target is actively engaged.
- It should keep the drone inside its wedge and near the configured ring radius.
- It should recover back into coverage after target loss.

### 4. Add local visual friendly classification scaffolding

- Add a perception hook for friendly marker detections.
- Associate marker detections with Norfair tracks.
- Mark linked tracks as `FRIENDLY`.
- Leave the LED-strip detector as a future implementation that plugs into the same classifier.

### 5. Add local claim logic

- Evaluate each unknown predicted track against wedge and breach rules.
- Lock onto one claimable target at a time.
- Add hysteresis so the drone does not bounce between targets near wedge boundaries.

## Safety Constraints

- Friendly keep-out remains a hard local constraint.
- If a target is ambiguous and outside the breach radius, do not engage.
- If local coverage recovery and intercept conflict, breach rules win only inside the breach zone.
- If multiple drones still converge on the same target, local avoidance must prefer maintaining
  safe spacing over perfect pursuit.

## Test Plan

### Simulation and replay

- Two-drone and three-drone launch-origin scenarios with no communication.
- Target enters a single wedge and is engaged by only the expected drone.
- Target crosses wedge boundary and hysteresis prevents rapid handoff thrashing.
- Two targets appear in different wedges and drones stay distributed.
- Friendly marker present and correctly excluded from interception.
- Ambiguous track near the perimeter is ignored.
- Ambiguous track inside breach radius is engaged.

### Bench and tethered

- Confirm patrol anchor generation differs across `drone_role_index`.
- Verify recovery back to patrol wedge after target loss.
- Verify friendly keep-out when a marked friendly is nearby.

### Flight

- Start with two drones and one unknown target.
- Validate launch spreading, patrol retention, and local intercept.
- Validate no-comms behavior when one drone is removed or fails to launch.

## Acceptance Criteria

- Drones do not all cluster at launch origin after startup.
- A locally visible unknown target is claimed by the correct drone under the wedge policy.
- Friendly tracks are excluded once marker detection is available.
- Coverage recovers after engagement without any runtime inter-drone communication.

## Assumptions

- There is no live inter-drone communication for coordination.
- The defended region is approximated by a launch-centered patrol ring rather than a preloaded map.
- Each drone can be configured with a unique `drone_role_index`.
- Visual markers are the first friendly-identification path; LED strip signaling is a future drop-in
  replacement or extension.
