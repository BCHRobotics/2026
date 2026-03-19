# FRC Team 2386 - 2026 Robot Code

[![Java](https://img.shields.io/badge/Java-17-blue)]()
[![WPILib](https://img.shields.io/badge/WPILib-2026-orange)]()

Robot code for Team 2386's 2026 robot. The project is built on WPILib 2026 and GradleRIO, with a MAXSwerve drivetrain, PhotonVision AprilTag pose fusion, a motorized ball intake, a climber, and a distance-aware shooter.

## Robot Overview

### Drivetrain

- REV MAXSwerve four-module drivetrain
- NavX-based field-relative driving
- Slew-rate-limited teleop drive
- Odometry plus `SwerveDrivePoseEstimator`
- PathPlanner AutoBuilder integration

### Vision

- PhotonVision AprilTag localization
- Multi-camera subsystem design with two cameras currently enabled
   - `ShooterLeft_Camera`
   - `ShooterRight_Camera`
- 2026 Rebuilt Andymark AprilTag field layout
- Vision measurement rejection by ambiguity, distance, translation delta, and rotation delta
- Vision measurements fused into drivetrain pose estimation

### Mechanisms

- Ball intake
   - extend motor on CAN 4
   - roller motor on CAN 6
   - calibration by current-spike detection at startup when needed
- Climber
   - climber motor on CAN 40
   - proximity switch on DIO 1
- Shooter
   - feeder motor on CAN 21
   - flywheel motors on CAN 22 and 23
   - SmartDashboard-tunable PID and feedforward values
   - target RPM derived from robot distance to the active alliance hub

## Controls

Driver controller type defaults to PS5 through `OIConstants.kDriverControllerType`.

### Driver PS5

- Left stick Y: drive forward and backward
- Left stick X: strafe left and right
- Right stick X: rotate chassis manually
- Square: hold `PointRearToAllianceHubCommand` to rotate so the rear points at the active alliance hub
- Circle: toggle intake roller on and off
- Triangle: zero drivetrain heading
- Cross: toggle intake arm between retracted and extended positions
- L1: start `ClimbCommand` using the selected climb start pose
- R2 or L2: run `ShootCommand` while held

### Driver Xbox Fallback

- Left stick Y: drive forward and backward
- Left stick X: strafe left and right
- Right stick X: rotate chassis manually
- X: hold `PointRearToAllianceHubCommand`
- B: toggle intake roller
- Y: zero drivetrain heading
- A: toggle intake arm position
- Left bumper: start `ClimbCommand`
- Left or right trigger: run `ShootCommand` while held

### Operator PS5

- Square: immediately kill shooter
- Circle: stop intake roller
- Triangle: extend climber while held
- Cross: retract climber while held
- R2: run `VortexSpeedShotCommand` while held
- L2: jiggle intake once

## Dashboard and Runtime Behavior

The code publishes or uses these major dashboard controls:

- `Auto Mode`
- `Field Pose`
- `Climb Start Pose`
- `Climb Command`
- `VisionTuningPath`
- shooter telemetry under `Shooter/...`
- climber telemetry under `Climber/...`
- vision telemetry under `Vision/...`
- drivetrain telemetry including `Gyro Heading`

Runtime behavior:

- teleop init calibrates the intake if it is not already calibrated
- autonomous also calibrates the intake before starting the selected auto
- drivetrain auto PID constants are selected from dashboard choosers before autonomous runs
- the vision subsystem is attached to the drivetrain for fused diagnostics and pose correction

## Autonomous

### Autos Exposed In The Dashboard Chooser

- `Practice_Auto`
- `Shooter-1_Auto`
- `Shooter-2_Auto`
- `Shooter-3_Auto`
- `Shooter-4_Auto`
- `Shooter-5_Auto`
- `Shooter-6_Auto`
- `Shooter-7_Auto`
- `Shooter-8_Auto`
- `Shooter-9_Auto`
- `Another_Practice`

### Additional Auto Files In The Repo

These exist under `src/main/deploy/pathplanner/autos`, but are not currently added to the SmartDashboard chooser:

- `Climber-1_Auto.auto` through `Climber-10_Auto.auto`
- `Practice_Auto.auto`
- `Shooter-1_Auto.auto` through `Shooter-9_Auto.auto`
- `Another_Practice.auto`

### Registered PathPlanner Named Commands

- `climber on`
- `shooter on`
- `shooter off`
- `intake on`
- `intake off`
- `jiggle on`

## Key Configuration

Important configuration lives in `src/main/java/frc/robot/Constants.java`.

Highlights from the current code:

- Java 17 target
- field-relative driving enabled by default
- rate limiting enabled by default
- wheelbase and track width are both `25.5 in`
- drive motor CAN IDs: `12, 10, 14, 16`
- turn motor CAN IDs: `13, 11, 15, 17`
- ball intake CAN IDs: `4, 6`
- climber CAN ID: `40`
- shooter CAN IDs: `21, 22, 23`
- enabled PhotonVision cameras: `ShooterLeft_Camera`, `ShooterRight_Camera`

Alliance hub centers used by the shooter and hub-pointing command are defined in `NavigationConstants`.

## Build and Deploy

### Prerequisites

- WPILib 2026
- Java 17
- VS Code with WPILib support

Vendor libraries currently referenced by the project:

- REVLib
- PhotonLib
- PathPlannerLib
- Studica NavX

### Build

```bash
./gradlew build
```

### Test

```bash
./gradlew test
```

### Deploy

Deploy with the WPILib VS Code commands or the usual driver-station workflow for your team.

## Repository Guides

Additional documentation in `docs/`:

- `docs/gyro.md`
- `docs/PHOTON_SETUP.md`
- `docs/vision_tuning.md`
- `docs/CameraTransform.md`
- `docs/APRILTAG_FIELD_LAYOUT.md`
- `docs/intake_commissioning_guide.md`
- `docs/ClimberFlowDiagram.md`
- `docs/joystick controls.md`
- `docs/SHUFFLEBOARD_ITEMS.md`
- `docs/FUNCTIONAL_SPECS.md`

## Code Layout

```text
src/main/java/frc/robot/
   Robot.java
   RobotContainer.java
   Constants.java
   Configs.java
   subsystems/
      Drivetrain.java
      Vision.java
      BallIntake.java
      Climber.java
      Shooter.java
      MAXSwerveModule.java
   commands/
      auto/
      ball/
      ballintake/
      climber/
      drivetrain/
      shooter/
      vision/

src/main/deploy/pathplanner/
   autos/
   paths/

utilities/
   nt_vision_monitor.py
   requirements.txt
   run_vision_monitor.sh
```

## Troubleshooting

If field-relative drive or auto alignment feels wrong, start here:

1. Verify `Gyro Heading` behaves correctly on the dashboard.
2. Re-zero heading with the robot pointed the intended field-forward direction.
3. Confirm PhotonVision camera names match the names in `VisionConstants`.
4. Confirm intake calibration completed before relying on intake position commands.
5. Compare odometry pose against vision-updated pose when tuning autonomous or vision filtering.

For subsystem-specific setup and tuning, use the matching files in `docs/`.
