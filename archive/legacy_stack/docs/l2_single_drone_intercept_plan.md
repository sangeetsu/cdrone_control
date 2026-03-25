# Level 2 Single-Drone Predictive Intercept Plan

## Summary

This document defines the next autonomy upgrade for one drone only. The goal is to replace the
current reactive follow behavior with predictive interception while keeping the existing Level 2
infrastructure intact:

- stereo tracking from `drone_vision_pkg`
- external pose/VIO contract for PX4
- monitor/dashboard support
- safety gates and manual takeover

The current implementation follows the target's current measured position. This plan upgrades the
stack so the drone steers toward a short-horizon predicted intercept point instead of trailing a
maneuvering target.

## Current Gaps

- The controller in `math_utils.py` is proportional pursuit against current target position.
- `stereo_tracker_node.py` estimates velocity by finite differencing positions, but does not
  estimate acceleration, turn rate, or future intercept geometry.
- Target ranking is based on distance and inbound motion, not predicted time-to-intercept.
- The behavior state machine does not have an intercept-specific phase.
- The dashboard does not expose prediction quality or intercept metrics.

## Target Architecture

### Perception and estimation

- Keep Norfair as the association and re-identification layer.
- Treat Norfair output as a measurement source, not the final predictor.
- Add a new `track_predictor_node` between raw tracked targets and the behavior manager.
- Run prediction in a stable ego-local frame so our own vehicle motion is removed before target
  motion is estimated.
- Use an interacting multiple model estimator (`IMM-EKF`) per active track.
- Start with these motion models:
  - constant velocity
  - constant acceleration
  - coordinated turn
- Publish filtered state, short-horizon predicted state, covariance/quality, and update age.

### Guidance and control

- Replace pure reactive pursuit with augmented proportional navigation (`APN`) for the intercept
  phase.
- Preserve a standoff controller after capture so the vehicle does not collide with the target.
- Split control into two regimes:
  - `INTERCEPT`: close time-to-target quickly using predicted target motion
  - `FOLLOW_STANDOFF`: once inside the capture envelope, regulate separation and alignment
- Use a receding horizon: recompute the intercept command at each tracker update.
- Keep output in the existing body-velocity command format so `mavros_velocity_node` and safety
  gates remain usable.

### Behavior state machine

- Update the state machine to:
  - `SEARCH`
  - `ALIGN`
  - `INTERCEPT`
  - `FOLLOW_STANDOFF`
  - `LOST_TARGET_HOLD`
  - `FAILSAFE_HOLD`
- `ALIGN` remains a short pre-close phase.
- `INTERCEPT` uses predicted target state and line-of-sight dynamics.
- `FOLLOW_STANDOFF` is used after closing distance and bearing error are both inside thresholds.
- `LOST_TARGET_HOLD` remains a hover/reacquire state.

## Interface Changes

### New messages

- Add `PredictedTrack.msg`
- Add `PredictedTrackArray.msg`

### `PredictedTrack` fields

- `stamp`
- `track_id`
- filtered target position in ego-local frame
- filtered target velocity in ego-local frame
- filtered target acceleration in ego-local frame
- predicted position at configured horizon
- `distance_m`
- `time_to_intercept_s`
- `prediction_age_s`
- `quality_score`
- covariance summary or equivalent confidence metric
- current target classification placeholder for future swarm work

### New topics

- `/cdrone/<drone_id>/perception/tracks_raw`
- `/cdrone/<drone_id>/perception/predicted_tracks`

The stereo tracker should keep publishing raw tracks. The new predictor node publishes the
intercept-ready target stream. The behavior manager should switch to consuming predicted tracks.

## Implementation Steps

### 1. Split raw tracking from prediction

- Rename the current track topic contract conceptually to "raw tracks".
- Leave `stereo_tracker_node.py` responsible for:
  - stereo matching
  - triangulation
  - Norfair ID continuity
  - raw 3D measurement output
- Do not push IMM logic into the stereo tracker.

### 2. Add `track_predictor_node`

- Subscribe to raw `TargetTrackArray`.
- Subscribe to ego pose/velocity needed to define a stable local frame.
- Maintain one IMM filter per live `track_id`.
- Expire filters when tracks go stale beyond a configurable timeout.
- Publish predicted tracks at the tracker rate.
- Provide per-track diagnostics:
  - last update age
  - active model weights
  - innovation magnitude
  - prediction horizon used

### 3. Update behavior ranking

- Replace the current score with predicted intercept cost.
- Primary ranking metric: `time_to_intercept_s`.
- Secondary tie-breakers:
  - prediction quality
  - target distance
  - target confidence
- Add lock hysteresis so a currently engaged target is not dropped because of small cost changes.

### 4. Replace reactive approach math

- Add an APN-based intercept utility in `math_utils.py`.
- Inputs:
  - filtered target position
  - filtered target velocity
  - filtered target acceleration when available
  - pursuer speed/acceleration limits
  - desired intercept horizon
- Outputs:
  - body-frame velocity command
  - yaw rate
  - intercept diagnostics
- Keep minimum-distance gating active after command synthesis.

### 5. Extend engagement state and dashboard visibility

- Add fields for:
  - `intercept_enabled`
  - `time_to_intercept_s`
  - `prediction_age_s`
  - `prediction_quality`
  - `line_of_sight_rate_rps`
  - `intercept_blocked_reason`
- Surface these in `FlightStatus` and the terminal dashboard.

## Safety Constraints

- If predicted tracks are stale, fall back to `LOST_TARGET_HOLD`.
- If prediction quality drops below threshold, do not start intercept.
- If VIO becomes stale, go to `FAILSAFE_HOLD`.
- Keep obstacle stop behavior as a hard gate outside the intercept law.
- Never allow positive closing velocity inside `min_safe_distance_m`.
- Manual takeover via `ALTCTL` remains the primary recovery path.

## Test Plan

### Bench and replay

- Replay synthetic target motions:
  - constant velocity
  - constant acceleration
  - coordinated turn
  - sudden heading reversal
  - stop-and-go motion
- Compare against the current controller on:
  - time-to-standoff
  - cross-track error
  - miss distance
  - overshoot

### Props-off integration

- Validate predictor topic freshness and track lifecycle.
- Confirm target lock hysteresis works when two tracks are near each other.
- Trigger stale prediction, stale VIO, and lost target conditions and verify hold behavior.
- Validate new dashboard fields update live.

### Flight

- Low-altitude long-range acquire in a clear test box.
- Verify the drone closes more directly than the current follow controller.
- Verify standoff capture remains stable without oscillation.
- Trigger target loss and manual takeover once each.

## Acceptance Criteria

- The drone no longer simply chases the current measured target point.
- A maneuvering target is intercepted faster than with the current Level 2 follow controller.
- Safety gates remain intact and manual takeover behavior is unchanged.
- The new predictor and intercept metrics are visible on the dashboard.

## Assumptions

- Norfair remains in the perception stack for association and re-ID.
- The first predictive controller is `APN + IMM-EKF`, not MPC.
- Ego pose aiding into PX4 remains unchanged from the current Level 2 work.
- Obstacle bypass is still out of scope for this step; the obstacle layer remains stop/hold only.
