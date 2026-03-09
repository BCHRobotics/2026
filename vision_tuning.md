# Vision Tuning Guide

This guide documents how to improve and tune AprilTag-based pose estimation for this robot.

It is written for the current implementation in:
- `src/main/java/frc/robot/subsystems/Vision.java`
- `src/main/java/frc/robot/subsystems/Drivetrain.java`
- `src/main/java/frc/robot/Constants.java`

The goal is to reduce noisy vision corrections, improve PathPlanner stability, and make vision measurements predictable enough to trust during autonomous.

## Summary

The current code now follows a safer pattern for FRC vision fusion:
- Each camera frame is processed once.
- Single-tag measurements are treated more conservatively than multi-tag measurements.
- Measurements are rejected when they are too far away, too ambiguous, or too far from the current fused pose.
- Measurement trust is scaled based on observed tag distance.
- SmartDashboard publishes acceptance and rejection diagnostics for tuning.

This is the same general strategy used by many strong FRC teams in 2026 style PhotonVision integrations: trust close multi-tag observations most, trust single-tag observations carefully, and never allow a bad vision solve to jerk the robot pose estimate.

## Current Tuning Values

These values are defined in `Constants.VisionConstants`.

### Base measurement trust

- `kSingleTagStdDevs = [0.35, 0.35, 0.45]`
- `kMultiTagStdDevs = [0.10, 0.10, 0.15]`
- `kDistanceWeight = 0.01`
- `kRotationDistanceWeight = 0.03`

Interpretation:
- Lower standard deviation means the pose estimator trusts vision more.
- Higher standard deviation means the pose estimator trusts odometry more.
- Multi-tag measurements should usually be trusted significantly more than single-tag measurements.

### Measurement rejection thresholds

- `kMaxAmbiguity = 0.2`
- `kMaxSingleTagDistance = 2.5 m`
- `kMaxMultiTagDistance = 4.0 m`
- `kMaxSingleTagPoseDeltaMeters = 0.75 m`
- `kMaxMultiTagPoseDeltaMeters = 1.50 m`
- `kMaxSingleTagRotationDeltaDegrees = 15 deg`
- `kMaxMultiTagRotationDeltaDegrees = 35 deg`

Interpretation:
- Single-tag solves should be close and stable before being accepted.
- Multi-tag solves can be accepted from farther away and with larger pose deltas.
- If a measurement disagrees too much with the current fused pose, it is probably bad and should be rejected.

## Dashboard Signals To Watch

The following SmartDashboard keys are the main tuning tools:

### Live tuning entries

All live-editable fusion tunables are published under:
- `Vision/Tuning/*`

Current keys:
- `Vision/Tuning/MaxAmbiguity`
- `Vision/Tuning/SingleTagXYStdDev`
- `Vision/Tuning/SingleTagThetaStdDev`
- `Vision/Tuning/MultiTagXYStdDev`
- `Vision/Tuning/MultiTagThetaStdDev`
- `Vision/Tuning/DistanceWeight`
- `Vision/Tuning/RotationDistanceWeight`
- `Vision/Tuning/MaxSingleTagDistance`
- `Vision/Tuning/MaxMultiTagDistance`
- `Vision/Tuning/MaxSingleTagPoseDeltaMeters`
- `Vision/Tuning/MaxMultiTagPoseDeltaMeters`
- `Vision/Tuning/MaxSingleTagRotationDeltaDegrees`
- `Vision/Tuning/MaxMultiTagRotationDeltaDegrees`

These values are seeded from `Constants.VisionConstants` at startup, then read live by the vision subsystem during pose acceptance and weighting. That means you can tune them on SmartDashboard without redeploying code.

### Global accepted measurement data

- `Vision/HasVisionPose`
- `Vision/LastAcceptedCamera`
- `Vision/LastAcceptedTagCount`
- `Vision/LastAcceptedAvgDistance`
- `Vision/LastAcceptedXYStdDev`
- `Vision/LastAcceptedThetaStdDev`
- `Vision/EstimatedPose/X`
- `Vision/EstimatedPose/Y`
- `Vision/EstimatedPose/Rotation`
- `Vision/PoseTimestamp`

### Per-camera diagnostics

For each configured camera name:
- `Vision/<camera name>/LastRejectReason`
- `Vision/<camera name>/PoseDeltaMeters`
- `Vision/<camera name>/RotationDeltaDegrees`

These values tell you why measurements are being rejected and whether the camera is producing consistent field poses.

## Recommended Tuning Order

Do not tune everything at once. Use this order.

1. Confirm camera calibration quality.
2. Confirm robot-to-camera transforms.
3. Tune with one camera enabled.
4. Tune single-tag trust and rejection thresholds.
5. Tune multi-tag trust and rejection thresholds.
6. Re-enable additional cameras one at a time.
7. Validate under motion.
8. Validate with PathPlanner autonomous routines.

## Step 1: Verify Calibration And Mounting

Before touching filter constants, verify the physical setup.

### Camera calibration

Each camera must be properly calibrated in PhotonVision.

Bad calibration causes:
- pose jitter at all ranges
- inconsistent tag distance estimates
- disagreement between cameras
- good-looking ambiguity values with bad actual field poses

If one camera consistently shows larger pose deltas or is frequently rejected while others are stable, suspect calibration first.

### Camera transform measurement

Measure each camera transform carefully:
- X: forward from robot center
- Y: left from robot center
- Z: upward from the floor
- roll, pitch, yaw: actual camera mounting orientation

Common failure cases:
- sign flipped on Y offset
- camera yaw off by 180 degrees or 90 degrees
- pitch left at zero when camera is actually tilted
- camera height measured from robot frame instead of floor reference assumptions

If the pose is consistently offset in one direction, fix transforms before tuning standard deviations.

## Step 2: Start With One Camera

Disable all but the best camera in `kCamerasEnabled`.

Pick the camera that:
- has the clearest view of field tags
- is least affected by robot mechanisms
- has the lowest vibration
- sees multi-tag layouts most often in autonomous

Why this matters:
- It isolates whether noise is coming from the filter or from one bad camera.
- It makes threshold tuning much faster.

## Step 3: Tune While Robot Is Stationary

Place the robot at a known field location and reset pose to that location.

Test three cases:
- close single-tag view
- close multi-tag view
- farther multi-tag view

Watch:
- `Vision/EstimatedPose/*`
- drivetrain fused pose on the field widget
- per-camera `PoseDeltaMeters`
- per-camera `RotationDeltaDegrees`
- `LastRejectReason`

### Desired stationary behavior

- The accepted pose should not wander significantly.
- The fused pose should settle instead of constantly twitching.
- Single-tag data may move slightly, but should not kick the estimator.
- Multi-tag data should look noticeably steadier than single-tag data.

### If stationary pose still jitters

Try these adjustments in order:

1. Increase `kSingleTagStdDevs`.
2. Decrease `kMaxSingleTagDistance`.
3. Decrease `kMaxSingleTagPoseDeltaMeters`.
4. Decrease `kMaxSingleTagRotationDeltaDegrees`.
5. If only one camera is bad, fix or disable that camera.

## Step 4: Tune Single-Tag Behavior

Single-tag solves are the most common source of noisy field pose updates.

Best practice:
- use them as a weak correction source
- allow them only when close
- reject them quickly when they disagree with odometry

### When to trust single-tag more

If close single-tag observations are obviously correct but the estimator is slow to converge:
- lower `kSingleTagStdDevs` slightly
- possibly raise `kMaxSingleTagPoseDeltaMeters` slightly

Make small changes only. Example:
- XY from `0.35` to `0.30`
- theta from `0.45` to `0.35`

### When to trust single-tag less

If the robot pose jitters or PathPlanner weaves during tag visibility:
- raise `kSingleTagStdDevs`
- lower `kMaxSingleTagDistance`
- lower `kMaxSingleTagPoseDeltaMeters`
- lower `kMaxSingleTagRotationDeltaDegrees`

Good conservative pattern:
- keep single-tag mostly for mild XY correction
- let gyro dominate heading unless multi-tag observations are present

## Step 5: Tune Multi-Tag Behavior

Multi-tag solves should be your main trusted vision source.

These usually deserve:
- lower XY standard deviations
- lower theta standard deviations than single-tag
- larger allowed acceptance distance
- larger allowed pose delta thresholds

### If close multi-tag solves are stable

You can usually trust them more by slightly lowering:
- `kMultiTagStdDevs`

Example:
- XY from `0.10` to `0.08`
- theta from `0.15` to `0.10`

### If far multi-tag solves start causing drift

Tighten one or more of:
- `kMaxMultiTagDistance`
- `kDistanceWeight`
- `kRotationDistanceWeight`

Interpretation:
- Higher distance weights reduce trust as target distance grows.
- Lower max multi-tag distance completely blocks far measurements.

## Step 6: Tune Distance Scaling

The current code scales measurement trust based on average camera-to-tag distance.

This is the correct signal to use, because it reflects the actual geometry seen by the camera.

### If far tags are still too influential

Increase:
- `kDistanceWeight`
- `kRotationDistanceWeight`

Suggested increments:
- `kDistanceWeight`: increase by `0.005`
- `kRotationDistanceWeight`: increase by `0.01`

### If the estimator ignores too many good far measurements

Decrease those values slightly, or increase:
- `kMaxMultiTagDistance`

Do not compensate for a bad camera by making the whole filter permissive.

## Step 7: Validate Under Motion

After stationary tuning, test these driving cases:
- slow straight drive toward a tag
- slow lateral strafe with tags in view
- slow rotation in place
- stop-and-go motion

### What to watch

- Does the fused pose step sideways when a tag appears?
- Does heading suddenly change while rotating?
- Does one camera frequently reject due to large pose delta?
- Does accepted vision lag enough to fight the robot during fast motion?

### Best practice during motion

If measurements are stable while stationary but bad in motion, that usually means one of these:
- timestamp/latency effects
- camera blur or exposure issues
- transform errors
- filter too trusting during fast turns

For this robot, the next likely improvement after basic tuning would be to reject or down-weight vision when:
- angular velocity is high
- translational acceleration is high
- the robot is crossing field elements that temporarily block tags

## Step 8: Validate With PathPlanner

PathPlanner is where noisy vision becomes obvious.

Run the same autonomous routine multiple times from the same starting pose.

### Healthy behavior

- repeated runs land in nearly the same place
- path following stays smooth when tags enter view
- the robot does not visibly snap sideways
- rotation remains controlled rather than oscillatory

### Symptoms and likely causes

If the robot weaves while following a path:
- single-tag trust is too high
- heading trust in vision is too high
- one camera is producing inconsistent solves

If the robot slowly drifts away from expected path:
- multi-tag trust may be too low
- odometry may be drifting more than expected
- accepted measurements may be too rare

If the robot snaps when a tag appears:
- pose delta thresholds are too loose
- standard deviations are too small
- transform or calibration is wrong

## Suggested Tuning Workflow At Competition

Use this quick process in the pit or practice field.

1. Enable one camera only.
2. Place robot at a known field mark.
3. Verify stationary pose error is reasonable.
4. Test close single-tag view.
5. Test close multi-tag view.
6. Drive short paths with tags entering and leaving view.
7. Re-enable next camera.
8. Repeat until all desired cameras are validated.

If one camera repeatedly causes issues, disable it rather than poisoning the whole fusion system.

## Recommended First Adjustments If Noise Persists

If the SmartDashboard still shows noisy accepted vision after the current code changes, make these changes in order:

1. Increase `kSingleTagStdDevs` to `[0.45, 0.45, 0.60]`.
2. Lower `kMaxSingleTagDistance` from `2.5` to `2.0`.
3. Lower `kMaxSingleTagPoseDeltaMeters` from `0.75` to `0.50`.
4. Lower `kMaxSingleTagRotationDeltaDegrees` from `15` to `10`.
5. If multi-tag remains stable, keep multi-tag settings as-is.

That is the safest next move for reducing PathPlanner variation without throwing away useful close multi-tag corrections.

## Recommended Future Improvements

If more robustness is needed, these are the next improvements worth implementing:

1. Publish per-camera accepted measurement counts and rejection counts.
2. Add a dashboard toggle to enable or disable each camera without recompiling.
3. Ignore or de-rate vision during high angular velocity.
4. Ignore or de-rate vision during high acceleration.
5. Use vision mostly for XY correction and de-rate theta heavily on single-tag frames.
6. Add logging for accepted and rejected measurements to DataLog for post-match review.

## Practical Rules Of Thumb

- Calibration and transforms matter more than filter constants.
- Multi-tag is usually worth trusting. Single-tag usually is not, unless close.
- If a measurement disagrees strongly with odometry, reject it first and investigate why.
- One bad camera can ruin a good pose estimator.
- Conservative vision integration wins more matches than aggressive but noisy vision integration.

## File Locations

- Vision fusion logic: `src/main/java/frc/robot/subsystems/Vision.java`
- Pose estimator integration: `src/main/java/frc/robot/subsystems/Drivetrain.java`
- Vision constants: `src/main/java/frc/robot/Constants.java`
- PhotonVision setup reference: `PHOTON_SETUP.md`
- Web diagnostics reference: `WEBSERVER_README.md`