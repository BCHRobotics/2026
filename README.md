# FRC Team 2386 - 2026 Robot Code

[![Java](https://img.shields.io/badge/Java-17-blue)]()
[![WPILib](https://img.shields.io/badge/WPILib-2026-orange)]()

Robot code for Team 2386's 2026 robot. The current codebase is built around a MAXSwerve drivetrain with NavX-based field-oriented control, PhotonVision AprilTag pose fusion, an active ball intake, a climber, and a distance-aware shooter.

## What Is In This Robot

### Drivetrain

- Four-module MAXSwerve drive using REV MAXSwerve hardware
- NavX gyro on MXP SPI for field-relative control and pose rotation
- Slew-rate-limited teleop drive
- Swerve odometry plus `SwerveDrivePoseEstimator`
- PathPlanner AutoBuilder integration for holonomic autonomous paths

### Vision

- PhotonVision AprilTag pose estimation
- Two active cameras in the current constants configuration
   - `ShooterLeft_Camera`
   - `ShooterRight_Camera`
- Vision measurement filtering by ambiguity, distance, and pose delta
- Vision updates fused into drivetrain pose estimation
- Field visualization and per-camera diagnostics on SmartDashboard

### Mechanisms

- Ball intake with:
   - extend/retract arm on CAN 4
   - roller on CAN 6
   - homing/calibration by current spike detection
- Climber with:
   - main motor on CAN 40
   - REV forward/reverse limit switches
   - proximity switch on DIO 1
- Shooter with:
   - feeder on CAN 21
   - dual flywheel motors on CAN 22 and 23
   - live-tunable velocity PID and feedforward
   - target RPM derived from estimated field distance to the hub

## Current Driver Controls

### Driver controller

Default driver controller type is PS5, configured in `OIConstants.kDriverControllerType`.

PS5 bindings:

- Left stick Y: drive forward/backward
- Left stick X: strafe left/right
- Right stick X: rotate chassis
- Square: hold `FacePointCommand` around the fixed field point `(11.945, 4.029)` at `2.8 m`
- Circle: toggle intake roller on/off
- Triangle: zero gyro heading
- Cross: toggle intake arm between retracted and extended positions
- L1: run `ClimbCommand` using the selected climb start pose
- R2 or L2: run shooter while held

Xbox fallback bindings exist in `RobotContainer`, but the active controller selection is PS5 unless constants are changed.

### Operator controller

Operator controller is a PS5 on port 1.

- Square: kill shooter immediately
- Circle: stop intake roller
- Triangle: extend climber while held
- Cross: retract climber while held

## Dashboard and Runtime Behavior

Published dashboard items used by the current robot code include:

- `Auto Mode` chooser
- `Climb Start Pose` chooser
- `Climb Command` button
- `VisionTuningPath` button
- `Gyro Heading`
- `Vision Field`
- climber telemetry under `Climber/...`
- shooter telemetry under `Shooter/...`
- vision telemetry under `Vision/...`

Startup behavior:

- teleop init schedules ball-intake calibration if the intake is not already calibrated
- autonomous also calibrates the intake before running the selected auto command
- driver heading zero is manual and bound to triangle on the PS5 controller

## Autonomous

The dashboard auto chooser currently exposes these PathPlanner autos:

- `Square Auto`
- `Tuning_auto`
- `Circle Auto`
- `Test Climber Centre`
- `Climber-1_Auto`
- `Climber-2_Auto`
- `Climber-3_Auto`
- `Climber-4_Auto`
- `Climber-5_Auto`
- `Climber-6_Auto`
- `N_Climber-7_Auto`
- `Climber-8_Auto`

Additional auto files exist under `src/main/deploy/pathplanner/autos`, including:

- `Another_Double_Circle_Auto.auto`
- `Double_Circle_Auto.auto`

Named PathPlanner command currently registered in code:

- `RunClimb`

## Key Configuration

Important constants live in `src/main/java/frc/robot/Constants.java`.

Highlights:

- field-relative driving is enabled by default
- rate limiting is enabled by default
- wheelbase and track width are both `24.5 in`
- drivetrain drive motor CAN IDs: `10, 12, 14, 16`
- drivetrain turn motor CAN IDs: `11, 13, 15, 17`
- current vision configuration enables two shooter-side cameras

One detail worth calling out: `DriveConstants.kGyroReversed` exists, but it is not referenced by the current drivetrain code. Changing that constant alone will not change heading behavior.

## Build and Deploy

### Prerequisites

- WPILib 2026
- Java 17
- VS Code with WPILib tools

Vendor libraries used by this project:

- REVLib
- PhotonLib
- PathPlannerLib
- Studica NavX

### Build

```bash
./gradlew build
```

### Deploy

Deploy with the WPILib VS Code command palette or your normal driver-station workflow.

## Vision Setup

Vision configuration is defined in `VisionConstants` and the `Vision` subsystem.

Current camera configuration in code:

- up to 4 cameras supported by the subsystem design
- 2 cameras enabled in the current constants array
- AprilTag field layout loaded from `AprilTagFields.k2026RebuiltAndymark`

Vision fusion notes:

- drivetrain pose rotation is still gyro-dominant
- vision acts as a correction source through `addVisionMeasurement(...)`
- measurements are rejected if ambiguity, tag distance, translation delta, or rotation delta exceed configured thresholds

For full setup and tuning instructions, use the dedicated docs below.

## Repository Guides

Top-level docs that match the current codebase:

- `docs/gyro.md`: where gyro values are used and how heading affects behavior
- `docs/PHOTON_SETUP.md`: PhotonVision setup and calibration guide
- `docs/vision_tuning.md`: vision tuning notes
- `docs/CameraTransform.md`: camera transform reference
- `docs/APRILTAG_FIELD_LAYOUT.md`: field layout notes
- `docs/intake_commissioning_guide.md`: intake bring-up and calibration notes
- `docs/ClimberFlowDiagram.md`: climber sequence reference
- `docs/joystick controls.md`: controller mapping notes
- `docs/SHUFFLEBOARD_ITEMS.md`: dashboard inventory
- `docs/FUNCTIONAL_SPECS.md`: higher-level system behavior notes

## Code Layout

```text
src/main/java/frc/robot/
   Robot.java
   RobotContainer.java
   Constants.java
   subsystems/
      Drivetrain.java
      Vision.java
      BallIntake.java
      Climber.java
      Shooter.java
      MAXSwerveModule.java
   commands/
      auto/
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
```

## Troubleshooting Pointers

If field-oriented driving feels wrong, start with these checks:

1. Confirm `Gyro Heading` on SmartDashboard behaves as expected.
2. Zero heading with the robot pointed in the intended field-forward direction.
3. Verify the active camera names match the PhotonVision configuration.
4. Check whether intake calibration succeeded before relying on extend/retract position commands.
5. Compare the robot's known field position against the vision-fused pose when tuning autonomous behavior.

For gyro-specific troubleshooting, see `docs/gyro.md`.
