# AdvantageScope Layout Configuration

**File:** `AdvantageScope 4-3-2026.json`  
**AdvantageScope Version:** 26.0.0  
**Window:** 1400×800 @ (162, 102)

---

## Tabs Overview

| # | Tab Title | Tab Type | Description |
|---|-----------|----------|-------------|
| 0 | 3D Field | Field3d | 3D visualization of robot pose, odometry, auto target, AprilTag positions, and camera positions on the 2026 FRC field |
| 1 | 2D Field | Field2d | Top-down 2D field view of fused robot pose, odometry ghost, active autonomous path, and auto target pose |
| 2 | Drivetrain | Line Graph | Chassis velocity components (Vx, Vy, Omega) on the left axis; heading and vision pose error on the right axis |
| 3 | Swerve Modules | Swerve | Real-time swerve module state vectors with robot heading overlay |
| 4 | Shooter | Line Graph | Shooter flywheel RPMs and target/ready thresholds on the left axis; shot distance and feeder speed on the right axis; charged/active/vortex state discrete overlays |
| 5 | Ball Intake | Line Graph | Extend position vs. target on the left axis; extend/run motor currents on the right axis; enabled/at-target/current-spike discrete overlays |
| 6 | Vision | Line Graph | Visible and accepted AprilTag counts and average tag distance on the left axis; vision pose error on the right axis; has-vision-pose discrete overlay |
| 7 | Match Overview | Line Graph | Cross-subsystem summary — drive speeds, shooter RPM vs. target, and intake position on the left axis; heading and visible tag count on the right axis; key state flags as discrete overlays |

---

## Tab 0 — 3D Field

**Type:** Field3d | **Field:** FRC:2026 Field

| Source | Log Key | Log Type | Description |
|--------|---------|----------|-------------|
| Robot | `/RealOutputs/Drivetrain/Pose3d` | `Pose3d` | Fused (vision + odometry) robot pose rendered as 2026 KitBot model |
| Ghost | `/RealOutputs/Drivetrain/OdometryPose3d` | `Pose3d` | Pure odometry pose (orange ghost) — shows drift vs. fused pose |
| Ghost | `/RealOutputs/Auto/TargetPose3d` | `Pose3d` | Current autonomous target pose (green ghost) |
| Game Piece | `/RealOutputs/Vision/TagPoses` | `Pose3d[]` | Detected AprilTag poses rendered as Fuel game pieces |
| Ghost | `/RealOutputs/Vision/CameraPoses` | `Pose3d[]` | Camera positions in world space (cyan ghost, model: Camera) |

---

## Tab 1 — 2D Field

**Type:** Field2d | **Field:** FRC:2026 Field | **Orientation:** 0 | **Size:** large

| Source | Log Key | Log Type | Description |
|--------|---------|----------|-------------|
| Robot | `/RealOutputs/Drivetrain/Pose` | `Pose2d` | Fused robot pose (primary robot icon) |
| Ghost | `/RealOutputs/Drivetrain/OdometryPose` | `Pose2d` | Odometry-only pose (orange ghost) — drift indicator |
| Trajectory | `/RealOutputs/Auto/ActivePath` | `Pose2d[]` | Active PathPlanner auto path (green, normal size) |
| Ghost | `/RealOutputs/Auto/TargetPose` | `Pose2d` | Current auto waypoint target (green ghost) |

---

## Tab 2 — Drivetrain

**Type:** Line Graph

### Left Axis — Velocity (m/s, rad/s)

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/RealOutputs/Drivetrain/ChassisSpeeds/VxMetersPerSecond` | `Double` | 🔵 `#4488ff` | Forward chassis velocity in m/s |
| `/RealOutputs/Drivetrain/ChassisSpeeds/VyMetersPerSecond` | `Double` | 🔴 `#ff4488` | Lateral chassis velocity in m/s |
| `/RealOutputs/Drivetrain/ChassisSpeeds/OmegaRadiansPerSecond` | `Double` | 🟠 `#ffaa00` | Rotational chassis velocity in rad/s |

### Right Axis — Heading / Error

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/RealOutputs/Drivetrain/HeadingDegrees` | `Double` | 🟢 `#aaffaa` | Robot heading from gyro in degrees |
| `/RealOutputs/Drivetrain/VisionPoseErrorMeters` | `Double` | 🟠 `#ff8800` | Distance error between vision and odometry poses in meters |

---

## Tab 3 — Swerve Modules

**Type:** Swerve | **Max Speed:** 4.8 m/s | **Robot Size:** 0.65 m × 0.65 m | **Orientation:** 1

| Source | Log Key | Log Type | Options | Description |
|--------|---------|----------|---------|-------------|
| States | `/RealOutputs/Drivetrain/ModuleStates` | `SwerveModuleState[]` | color `#ff4444`, arrangement `0,1,2,3` | Measured module speeds and angles for all 4 swerve modules |
| Rotation | `/RealOutputs/Drivetrain/HeadingDegrees` | `Double` | units: degrees | Robot heading overlay on swerve diagram |

---

## Tab 4 — Shooter

**Type:** Line Graph

### Left Axis — RPM

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/SmartDashboard/Shooter/Shooter1RPM` | `Double` | 🔴 `#ff4444` | Flywheel 1 measured speed in RPM |
| `/SmartDashboard/Shooter/Shooter2RPM` | `Double` | 🟠 `#ff8844` | Flywheel 2 measured speed in RPM |
| `/SmartDashboard/Shooter/TargetRPM` | `Double` | 🟢 `#44ff44` | Target/commanded RPM setpoint |
| `/SmartDashboard/Shooter/ReadyRPM` | `Double` | 🟡 `#ffff44` | Ready-to-shoot RPM threshold |

### Right Axis — Distance / Feeder

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/SmartDashboard/Shooter/Distance` | `Double` | 🔵 `#44aaff` | Distance to target used for shot calculation (meters) |
| `/SmartDashboard/Shooter/AppliedFeederSpeed` | `Double` | 🟣 `#cc88ff` | Applied feeder motor speed (−1 to 1) |

### Discrete (Boolean) Overlays

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/SmartDashboard/Shooter/Charged` | `Boolean` | 🟢 `#00ff88` | True when shooter is at target RPM and ready to fire |
| `/SmartDashboard/Shooter/Active` | `Boolean` | 🔴 `#ff4444` | True when shooter is actively running |
| `/SmartDashboard/Shooter/VortexSpeedShotActive` | `Boolean` | 🟠 `#ff8800` | True when a Vortex-speed shot profile is active |

---

## Tab 5 — Ball Intake

**Type:** Line Graph

### Left Axis — Position (encoder units)

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/SmartDashboard/BallIntake/ExtendPosition` | `Double` | 🔵 `#44aaff` | Actual extend mechanism position |
| `/SmartDashboard/BallIntake/TargetExtendPosition` | `Double` | 🟢 `#44ff44` | Target/commanded extend position |

### Right Axis — Current (A)

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/SmartDashboard/BallIntake/ExtendCurrent` | `Double` | 🟠 `#ff8844` | Raw extend motor current draw in amps |
| `/SmartDashboard/BallIntake/FilteredExtendCurrent` | `Double` | 🟡 `#ffff44` | Low-pass filtered extend current (used for spike detection) |
| `/SmartDashboard/BallIntake/RunCurrent` | `Double` | 🟣 `#cc88ff` | Roller/run motor current draw in amps |

### Discrete (Boolean) Overlays

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/SmartDashboard/BallIntake/ExtendEnabled` | `Boolean` | 🔵 `#44aaff` | True when the extend mechanism is enabled |
| `/SmartDashboard/BallIntake/AtTarget` | `Boolean` | 🟢 `#44ff44` | True when extend position is within tolerance of target |
| `/SmartDashboard/BallIntake/current spike` | `Boolean` | 🔴 `#ff4444` | True when a current spike (ball contact) has been detected |

---

## Tab 6 — Vision

**Type:** Line Graph

### Left Axis — Tag Counts / Distance

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/RealOutputs/Vision/VisibleTagCount` | `Integer` | 🟢 `#44ff44` | Number of AprilTags currently visible across all cameras |
| `/RealOutputs/Vision/LastAcceptedTagCount` | `Integer` | 🔵 `#44aaff` | Number of tags in the last accepted vision pose estimate |
| `/RealOutputs/Vision/LastAcceptedAvgTagDistanceMeters` | `Double` | 🟠 `#ffaa44` | Average distance to accepted tags in meters (lower = more accurate) |

### Right Axis — Pose Error

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/RealOutputs/Drivetrain/VisionPoseErrorMeters` | `Double` | 🔴 `#ff4488` | Distance between vision pose estimate and odometry pose in meters |

### Discrete (Boolean) Overlays

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/RealOutputs/Vision/HasVisionPose` | `Boolean` | 🟢 `#44ff44` | True when a valid vision pose has been accepted this cycle |

---

## Tab 7 — Match Overview

**Type:** Line Graph (cross-subsystem summary)

### Left Axis — Speeds / Position

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/RealOutputs/Drivetrain/ChassisSpeeds/VxMetersPerSecond` | `Double` | 🔵 `#4488ff` | Forward drive speed (m/s) |
| `/RealOutputs/Drivetrain/ChassisSpeeds/VyMetersPerSecond` | `Double` | 🔴 `#ff4488` | Lateral drive speed (m/s) |
| `/SmartDashboard/Shooter/Shooter1RPM` | `Double` | 🟠 `#ff8844` | Shooter flywheel 1 speed (RPM) |
| `/SmartDashboard/Shooter/TargetRPM` | `Double` | 🟢 `#44ff44` | Shooter target RPM |
| `/SmartDashboard/BallIntake/ExtendPosition` | `Double` | 🟣 `#cc88ff` | Ball intake extend position |

### Right Axis — Heading / Tag Count

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/RealOutputs/Drivetrain/HeadingDegrees` | `Double` | 🟢 `#aaffaa` | Robot heading in degrees |
| `/RealOutputs/Vision/VisibleTagCount` | `Integer` | 🔵 `#44aaff` | Number of visible AprilTags |

### Discrete (Boolean) Overlays

| Log Key | Log Type | Color | Description |
|---------|----------|-------|-------------|
| `/SmartDashboard/Shooter/Charged` | `Boolean` | 🟢 `#00ff88` | Shooter charged and ready |
| `/SmartDashboard/Shooter/Active` | `Boolean` | 🔴 `#ff4444` | Shooter actively firing |
| `/RealOutputs/Vision/HasVisionPose` | `Boolean` | 🟢 `#44ff44` | Vision pose accepted this cycle |
| `/SmartDashboard/BallIntake/ExtendEnabled` | `Boolean` | 🔵 `#44aaff` | Intake extend mechanism enabled |
| `/SmartDashboard/BallIntake/AtTarget` | `Boolean` | 🟢 `#aaffaa` | Intake at target position |

---

## Sidebar — Auto-Expanded Keys

The following log tree nodes are expanded by default in the sidebar:

- `/PathPlanner`
- `/RealOutputs`
- `/RealOutputs/Drivetrain`
- `/RealOutputs/Vision`
- `/RealOutputs/Auto`
- `/SmartDashboard`
- `/SmartDashboard/Shooter`
- `/SmartDashboard/BallIntake`
