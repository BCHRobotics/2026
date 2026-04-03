# Pose Objects Published to AdvantageScope (Pose2d & Pose3d)

All poses are published via `Logger.recordOutput()` (AdvantageKit) and appear under the `NT:/AdvantageKit/RealOutputs/` prefix in AdvantageScope.

> **Note:** A separate `Vision/Field` widget (a `Field2d`) is also published to SmartDashboard/Shuffleboard via `SmartDashboard.putData()` in `Vision.java`. This is **not** an AdvantageKit output and does not appear in the AdvantageScope log — it is Shuffleboard-only.

---

## All Published Pose2d Objects

| AdvantageScope Key | Type | Source File | Published In | Description |
|---|---|---|---|---|
| `Drivetrain/Pose` | `Pose2d` (single) | `Drivetrain.java` | `logAdvantageScopeData()` | Vision-fused robot pose from the pose estimator. The 2D counterpart of `Drivetrain/Pose3d`. Used by PathPlanner, ClimbCommand, and hub-distance calculations. |
| `Drivetrain/OdometryPose` | `Pose2d` (single) | `Drivetrain.java` | `logAdvantageScopeData()` | Wheel-odometry-only robot pose, before vision corrections are applied. The 2D counterpart of `Drivetrain/OdometryPose3d`. |
| `Auto/TargetPose` | `Pose2d` (single) | `RobotContainer.java` | `configurePathPlannerLogging()` | PathPlanner's current target waypoint pose as a flat 2D pose. Published alongside `Auto/TargetPose3d` from the same callback. |
| `Auto/ActivePath` | `Pose2d[]` (array) | `RobotContainer.java` | `configurePathPlannerLogging()` | The full sequence of waypoints in the currently-executing PathPlanner path. Updated whenever the active path changes via `setLogActivePathCallback`. |

---

## Pose2d Subsystem Dependency Map

| Subsystem / Feature | Pose2d Key(s) Used | How It's Used |
|---|---|---|
| **Drivetrain** | `Drivetrain/Pose`, `Drivetrain/OdometryPose` | Self-published each loop for visualization and diagnostics. `VisionPoseErrorMeters` is derived by comparing the two translations. |
| **PathPlanner (auto)** | `Auto/TargetPose`, `Auto/ActivePath` | Published by PathPlanner callbacks in `RobotContainer`. PathPlanner reads `drivetrain.getPose()` (`Pose2d`) internally for path following. |
| **Shooter / Distance-to-Hub** | *(consumes `Drivetrain/Pose` indirectly)* | `Shooter.getHubDistance()` calls `drivetrain.getPose()` which is the same pose logged as `Drivetrain/Pose`. Result is a plain `double` on SmartDashboard — no Pose2d is published by Shooter. |
| **Climb Positioning (`ClimbCommand`)** | *(consumes `Drivetrain/Pose` indirectly)* | `ClimbCommand.driveToPose()` calls `drivetrain.getPose()` each loop to close the position PID. Target poses (`ClimbConstants.kBlue/RedLeft/RightStartPose`) are `Pose2d` constants — **not published** to AdvantageKit. |
| **PointRearToAllianceHubCommand** | *(consumes `Drivetrain/Pose` indirectly)* | Calls `drivetrain.getPose()` each loop to compute the bearing to the alliance hub. No Pose2d published. |
| **AlignToAprilTagCommand** | *(consumes `Drivetrain/Pose` indirectly)* | Calls `drivetrain.getPose()` for PID error calculation. No Pose2d published. |
| **Vision** | *(produces, does not consume)* | `Vision.periodic()` computes `Pose2d visionPose` from each camera estimate and pushes it into `drivetrain.addVisionMeasurement()`, which feeds `Drivetrain/Pose`. Also updates the SmartDashboard `Field2d` widget. |

---

## All Published Pose3d Objects

| AdvantageScope Key | Type | Source File | Published In | Description |
|---|---|---|---|---|
| `Drivetrain/Pose3d` | `Pose3d` (single) | `Drivetrain.java` | `logAdvantageScopeData()` | Vision-fused robot pose (pose estimator). Flat on field (Z=0, no tilt). |
| `Drivetrain/OdometryPose3d` | `Pose3d` (single) | `Drivetrain.java` | `logAdvantageScopeData()` | Wheel-odometry-only robot pose, before vision correction. Flat on field (Z=0, no tilt). |
| `Vision/RobotPoses` | `Pose3d[]` (array) | `Vision.java` | `periodic()` | All accepted vision-estimated robot poses from camera frames this loop cycle. Includes 3D camera-derived height. |
| `Vision/CameraPoses` | `Pose3d[]` (array) | `Vision.java` | `periodic()` via `getCameraFieldPoses()` | 3D field-space positions of each camera, computed by applying each camera's robot-relative transform to the current drivetrain pose. |
| `Vision/TagPoses` | `Pose3d[]` (array) | `Vision.java` | `periodic()` via `getDetectedTagFieldPoses()` | 3D field-space poses of all AprilTags currently detected by any camera (pulled from the AprilTag field layout). |
| `Auto/TargetPose3d` | `Pose3d` (single) | `RobotContainer.java` | `configurePathPlannerLogging()` | PathPlanner's current target waypoint pose, lifted from 2D to 3D (Z=0, flat). Published via `PathPlannerLogging.setLogTargetPoseCallback`. |

---

## Pose3d Subsystem Dependency Map

| Subsystem / Feature | Pose3d Key(s) Used | How It's Used |
|---|---|---|
| **Drivetrain** | `Drivetrain/Pose3d`, `Drivetrain/OdometryPose3d` | Self-published for visualization. `Pose3d` = fused pose; `OdometryPose3d` = raw wheel odometry. |
| **Vision** | `Vision/RobotPoses`, `Vision/CameraPoses`, `Vision/TagPoses` | Self-published. `Vision.periodic()` feeds accepted vision measurements back into `Drivetrain.addVisionMeasurement()`, closing the fusion loop. |
| **PathPlanner (auto)** | `Auto/TargetPose3d` | Published by PathPlanner's target-pose logging callback in `RobotContainer`. Used for visualizing the active waypoint in AdvantageScope during autonomous. |
| **Shooter / Distance-to-Hub** | *(none — Pose3d not used)* | `Shooter.getHubDistance()` calls `drivetrain.getPose()` (`Pose2d`) and computes a 2D `Translation2d` distance to `NavigationConstants.kBlueAllianceHubCenter` / `kRedAllianceHubCenter`. **No Pose3d object is published or consumed.** Distance is reported to SmartDashboard only (`Shooter/Distance`). |
| **Climb Positioning (`ClimbCommand`)** | *(none — Pose3d not used)* | `ClimbCommand` drives to `ClimbConstants.kBlueLeftStartPose` / `kRedLeftStartPose` etc. using `Pose2d` references and `drivetrain.getPose()`. Target poses are sent to SmartDashboard (`ClimbCommand/` keys) but **no Pose3d is published to AdvantageKit**. The command reads `Drivetrain/Pose3d` indirectly via `drivetrain.getPose()` which is the same fused pose. |
| **PointRearToAllianceHubCommand** | *(none — Pose3d not used)* | Uses `drivetrain.getPose()` (`Pose2d`) to calculate the bearing to the alliance hub center. Heading-only control; no Pose3d. |
| **AlignToAprilTagCommand** | *(none — Pose3d not used)* | Uses `vision.getFieldLayout().getTagPose()` (an `Optional<Pose3d>` from WPILib's field layout) internally to set PID setpoints, but **does not publish it** via AdvantageKit. Reads drivetrain pose as `Pose2d`. |

---

## Notes on Specific Features

### PathPlanner
- **Pose2d keys published:** `Auto/TargetPose` (single), `Auto/ActivePath` (array).
- **Pose3d key published:** `Auto/TargetPose3d` (same target pose, lifted to 3D with Z=0).
- Both are published in `RobotContainer.configurePathPlannerLogging()` via PathPlanner's `setLogTargetPoseCallback` and `setLogActivePathCallback` hooks.
- PathPlanner reads `drivetrain.getPose()` (`Pose2d`) internally for path following — the same value logged as `Drivetrain/Pose`.

### Distance-to-Hub (Shooter)
- **Does not publish any Pose2d or Pose3d to AdvantageKit.**
- `Shooter.getHubDistance()` calls `drivetrain.getPose()` (the `Drivetrain/Pose` value) and computes a 2D `Translation2d` distance to `NavigationConstants.kBlueAllianceHubCenter` / `kRedAllianceHubCenter`.
- The result is published to SmartDashboard as a plain `double` (`Shooter/Distance`), not to AdvantageKit.
- To see hub distance in AdvantageScope, add `Logger.recordOutput("Shooter/HubDistanceMeters", getHubDistance())` in `Shooter.publishTelemetry()`.

### Climb Positioning (ClimbCommand)
- **Pose2d keys consumed (indirect):** `Drivetrain/Pose` (via `drivetrain.getPose()`).
- **Does not use any logged Pose3d or Pose2d directly.**
- `ClimbCommand` drives to fixed `Pose2d` constants (`ClimbConstants.kBlue/RedLeft/RightStartPose`) and a dynamically computed `targetPose` (start pose + a forward offset via `transformBy`).
- Phase and current-pose info is published to SmartDashboard (`ClimbCommand/Phase`, `ClimbCommand/CurrentPoseX/Y`, `ClimbCommand/AtStartPose`) but **not to AdvantageKit**.
- If you want Pose2d/Pose3d visualization of climb targets in AdvantageScope, add inside `ClimbCommand.initialize()`:
  ```java
  Logger.recordOutput("ClimbCommand/StartPose", startPose);
  Logger.recordOutput("ClimbCommand/TargetPose", targetPose);
  Logger.recordOutput("ClimbCommand/TargetPose3d", new Pose3d(targetPose));
  ```

---

## AdvantageScope Recommended 2D Field Config

For a 2D field overlay in AdvantageScope:

| Object | Key | Type |
|---|---|---|
| Robot (fused) | `RealOutputs/Drivetrain/Pose` | `Robot` |
| Robot (odometry only) | `RealOutputs/Drivetrain/OdometryPose` | `Ghost` |
| PathPlanner target waypoint | `RealOutputs/Auto/TargetPose` | `Ghost` |
| PathPlanner active path | `RealOutputs/Auto/ActivePath` | `Trajectory` |

---

## AdvantageScope Recommended 3D Field Config

For a working 3D robot visualization in AdvantageScope, configure the following objects on the 3D field:

| Object | Key | Type |
|---|---|---|
| Robot (fused) | `RealOutputs/Drivetrain/Pose3d` | `Robot` |
| Robot (odometry only) | `RealOutputs/Drivetrain/OdometryPose3d` | `Ghost` |
| Vision estimates | `RealOutputs/Vision/RobotPoses` | `Ghost` |
| Camera positions | `RealOutputs/Vision/CameraPoses` | `Component` |
| Detected AprilTags | `RealOutputs/Vision/TagPoses` | `AprilTag` |
| PathPlanner target waypoint | `RealOutputs/Auto/TargetPose3d` | `Ghost` |

