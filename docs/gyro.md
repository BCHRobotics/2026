# Gyro Usage Map and Troubleshooting

This document identifies the places in the robot code that use gyro-derived heading and explains how those values affect robot behavior.

The main goal is troubleshooting unexpected field-oriented control behavior. In this codebase, that usually means one of these is wrong:

- the raw NavX heading is wrong
- the heading was zeroed at the wrong time
- the pose estimator is carrying a bad rotation
- a command is using field-relative motion while assuming the gyro heading is trustworthy

## Primary gyro source

### `src/main/java/frc/robot/subsystems/Drivetrain.java`

This is the authoritative gyro interface for the robot.

- `public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);`
  - Creates the NavX gyro on MXP SPI.
- `getRotation2d()`
  - Returns `Rotation2d.fromDegrees(gyro.getAngle())`.
  - This is the raw heading source used by the drivetrain.
  - `gyro.getAngle()` is continuous and can exceed `360` or `-360`.
- `getHeading()`
  - Returns `getRotation2d().getDegrees()`.
  - Used anywhere a plain degree value is needed.
- `zeroHeadingOnly()`
  - Calls `gyro.reset()` so the robot's current facing direction becomes `0 deg`.
  - Immediately resets both `odometry` and `poseEstimator` with `Rotation2d.fromDegrees(0.0)`.
  - This is important: zeroing the gyro also changes the rotation being tracked by drivetrain pose systems.
- `zeroHeading()`
  - Just calls `zeroHeadingOnly()`.

## Direct field-oriented control use

### `src/main/java/frc/robot/subsystems/Drivetrain.java`

`drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit)` is the main field-oriented control path.

- When `fieldRelative` is `true`, the code uses:
  - `ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d())`
- That means the driver's forward/left/right commands are interpreted in field coordinates and then converted into robot-relative wheel commands using the current gyro heading.

If the gyro heading is wrong here, the robot will:

- drive at the wrong angle relative to the field
- appear rotated from the driver's perspective
- translate diagonally when the stick is pushed straight
- feel correct in robot-relative mode but wrong in field-relative mode

### `src/main/java/frc/robot/RobotContainer.java`

The default teleop drive command is configured here.

- `TeleopDriveCommand` is installed as the drivetrain default command.
- It is passed `OIConstants.kFieldRelative`.
- In `Constants.java`, `OIConstants.kFieldRelative = true`.

That means normal driver control is always field-oriented unless code is changed.

### `src/main/java/frc/robot/commands/drivetrain/TeleopDriveCommand.java`

This command does not read the gyro directly, but it continuously calls:

- `m_drivetrain.drive(xSpeed, ySpeed, rotation, m_fieldRelative, m_rateLimit)`

Because `m_fieldRelative` is set to `true` by default, this command is one of the main places where a bad gyro is visible to drivers.

## Gyro zeroing path

### `src/main/java/frc/robot/RobotContainer.java`

The driver button binding for heading reset is here.

- PS5: triangle
- Xbox: Y
- Both schedule `new ZeroHeadingCommand(robotDrive)`

### `src/main/java/frc/robot/commands/drivetrain/ZeroHeadingCommand.java`

- Calls `m_drivetrain.zeroHeading()` in `initialize()`
- Finishes immediately

Operational meaning:

- whatever direction the robot is facing when the driver presses the button becomes field forward
- if the robot is not physically pointed where the team expects, field-oriented driving will feel wrong immediately after zeroing

## Drivetrain odometry and pose estimation use the gyro

### `src/main/java/frc/robot/subsystems/Drivetrain.java`

The gyro is not only for teleop field orientation. It also feeds every drivetrain pose calculation.

- `SwerveDriveOdometry odometry = new SwerveDriveOdometry(..., getRotation2d(), ...)`
- `SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(..., getRotation2d(), ...)`
- `periodic()` updates both with `getRotation2d()`
- `resetPose(...)` resets both using the current heading from `getRotation2d()`
- `getPose()` returns the vision-fused pose estimator result
- `getOdometryPose()` returns wheel-odometry-only pose

Why this matters:

- a wrong gyro heading does not only rotate the robot's driving frame
- it also corrupts the estimated field pose used by autonomous, vision checks, climb alignment, and any command that asks for `getPose()`

## Dominant contributor to pose rotation

For this robot, the dominant contributor to pose rotation is the gyro, not vision.

Why:

- `odometry` and `poseEstimator` are both updated every drivetrain loop with `getRotation2d()`
- `getRotation2d()` is just `Rotation2d.fromDegrees(gyro.getAngle())`
- every field-relative drive conversion also uses that same drivetrain heading immediately
- vision measurements are only occasional corrections added through `addVisionMeasurement(...)`

So in normal operation, pose rotation is gyro-led and vision is correction-only.

### What vision can still do

Vision can still influence the estimated pose rotation because `Vision.java` passes a full `Pose2d` into `poseEstimator.addVisionMeasurement(...)`, and that pose includes theta.

However, in this codebase vision is usually the secondary contributor because:

- the estimator is continuously fed gyro heading every loop
- vision updates arrive less often than drivetrain updates
- vision measurements can be rejected before they are fused
- even accepted vision updates are weighted by standard deviations, so they act like corrections rather than the main heading source

### Practical interpretation

If pose rotation is suddenly wrong during driver control, assume gyro first.

If pose rotation drifts or jumps only when tags are seen, then vision may be contributing, but it is still correcting around a gyro-defined baseline.

In other words:

- fast, continuous rotation truth comes from the gyro
- slow, intermittent rotation correction can come from vision
- if the gyro baseline is wrong, most commands will still behave wrong even with working vision

### SmartDashboard telemetry

`Drivetrain.periodic()` publishes:

- `SmartDashboard.putNumber("Gyro Heading", getHeading())`

This is the first value to watch when field-oriented driving feels wrong.

## Autonomous path following depends on gyro-derived pose

### `src/main/java/frc/robot/subsystems/Drivetrain.java`

`configureAutoBuilder(...)` passes these drivetrain hooks into PathPlanner:

- `this::getPose`
- `this::resetPose`
- `this::getRobotRelativeSpeeds`
- `this::driveRobotRelative`

`getPose()` is gyro-backed through the pose estimator, so path following depends on correct heading even when the autonomous command itself does not call `getHeading()` directly.

If heading is wrong, autonomous symptoms can include:

- the robot starting a path with the wrong orientation
- translation corrections that look rotated relative to the field
- overshoot or sideways drift during path tracking

## Commands that read gyro heading directly

### `src/main/java/frc/robot/commands/drivetrain/VisionTuningPath.java`

Direct heading usage:

- captures `heldHeadingDegrees = drivetrain.getHeading()` during `initialize()`
- rotation PID uses `rotationController.calculate(drivetrain.getHeading())`
- field-relative chassis conversion uses `Rotation2d.fromDegrees(drivetrain.getHeading())`

How heading is used:

- the command locks the robot to its initial heading while driving between waypoints
- if gyro heading drifts or was zeroed badly, the robot will try to hold the wrong angle for the entire test path

### `src/main/java/frc/robot/commands/climber/ClimbCommand.java`

Direct heading usage:

- converts field-relative PID outputs into robot-relative chassis speeds with:
  - `ChassisSpeeds.fromFieldRelativeSpeeds(..., Rotation2d.fromDegrees(drivetrain.getHeading()))`

Indirect heading usage in the same command:

- `drivetrain.getPose()` is used for current translation and rotation
- heading error checks use `currentPose.getRotation().getDegrees()`

How heading is used:

- the robot drives to climb start and approach poses in field coordinates
- if the gyro is wrong, the robot can translate in the wrong direction while trying to approach the correct field pose

## Commands that use pose rotation derived from the gyro

These locations do not call `gyro.getAngle()` directly, but they use `drivetrain.getPose()` or call `drive(..., fieldRelative=true, ...)`. That still depends on gyro heading.

### `src/main/java/frc/robot/commands/drivetrain/FacePointCommand.java`

- reads `Pose2d robotPose = m_drivetrain.getPose()`
- uses `robotPose.getRotation()` when radius is near zero
- commands `m_drivetrain.drive(..., true, false)` in field-relative mode

How heading is used:

- the robot tries to orbit a field point while facing away from it
- if heading is wrong, the tangent drive and facing direction can both be wrong

### `src/main/java/frc/robot/commands/drivetrain/GoToPositionCommand.java`

- reads `m_drivetrain.getPose()`
- uses `currentPose.getRotation().getDegrees()` for rotation PID
- commands `m_drivetrain.drive(..., true, false)`

How heading is used:

- the command navigates to a field pose
- a wrong gyro causes both bad pose estimation and wrong field-relative drive conversion

### `src/main/java/frc/robot/commands/drivetrain/GoToPositionRelativeCommand.java`

- reads `m_drivetrain.getPose()`
- uses `currentPose.getRotation().getDegrees()` for rotation PID
- commands `m_drivetrain.drive(..., true, false)`

How heading is used:

- same failure mode as `GoToPositionCommand`, but with an alliance-relative target converted to field coordinates first

### `src/main/java/frc/robot/commands/vision/AlignToAprilTagCommand.java`

- reads `m_drivetrain.getPose()`
- uses `currentPose.getRotation().getDegrees()` for rotation PID
- commands `m_drivetrain.drive(..., true, false)`

How heading is used:

- the robot drives to the field pose of a selected AprilTag
- bad gyro data can make alignment look like vision is wrong when the real problem is the drivetrain field frame

## Code that indirectly depends on the gyro through `getPose()`

These places are not the first suspects for bad driver field orientation, but they can show secondary symptoms when heading is wrong.

### `src/main/java/frc/robot/commands/auto/DriveForwardCommand.java`

- stores `m_drivetrain.getPose().getX()` on start
- later compares current `getPose().getX()` to determine distance traveled

Why gyro matters:

- although the command drives robot-relative, the distance check uses estimated field X
- if heading is wrong, the projected field X distance can be inaccurate

### `src/main/java/frc/robot/subsystems/Shooter.java`

- `getHubDistance()` uses `drivetrain.getPose().getTranslation().getDistance(...)`

Why gyro matters:

- swerve pose translation accuracy depends on heading
- a heading error can distort the estimated shooting distance and therefore the chosen RPM

### `src/main/java/frc/robot/subsystems/Vision.java`

Indirect gyro-backed pose usage:

- compares AprilTag pose estimates to `drivetrain.getPose()`
- computes `rotationDeltaDegrees` between vision pose and drivetrain pose
- uses `drivetrain.getPose()` when reporting tag distances
- uses `drivetrain.getPose()` in `getTagNavigationInfo(...)`

Why gyro matters:

- if the drivetrain heading is wrong, vision pose rejection may trigger because the fused pose and camera estimate disagree in rotation
- this can make the robot appear to have a vision problem when the root issue is heading or pose estimation

## Configuration notes that matter for troubleshooting

### `src/main/java/frc/robot/Constants.java`

- `OIConstants.kFieldRelative = true`
  - teleop is intended to be field-oriented by default
- `DriveConstants.kGyroReversed = true`
  - this constant is currently not referenced anywhere in the codebase
  - changing it will not change robot behavior unless code is updated to use it

This is important because someone reading constants may assume the gyro sign can be fixed there, but in the current code it cannot.

## Practical troubleshooting flow for bad field-oriented behavior

1. Put the robot on the floor pointed in a known field-forward direction.
2. Watch `Gyro Heading` on SmartDashboard.
3. Press the zero-heading button once and confirm the heading becomes approximately `0 deg`.
4. Rotate the robot by hand and confirm the heading changes in the expected direction.
5. Test simple teleop translation with no autonomous or vision command running.

If teleop field-oriented driving is wrong immediately:

- inspect `Drivetrain.getRotation2d()` and `Drivetrain.drive(...)`
- confirm the zero-heading button was pressed while the robot was actually facing field forward
- confirm NavX mounting orientation matches the assumption in the comment above `getRotation2d()`

If teleop seems mostly fine, but autonomous or assisted commands are wrong:

- inspect `getPose()` consumers first
- check whether `zeroHeading()` was used after the pose was already established
- compare `getPose()` against known robot position on the field
- inspect vision rejection logs and `RotationDeltaDegrees` dashboard values

If only climb or waypoint commands are wrong:

- inspect commands using `ChassisSpeeds.fromFieldRelativeSpeeds(...)`
- those commands rely on the gyro both for current pose rotation and for field-to-robot conversion

## Most likely root causes for this codebase

- The gyro was zeroed while the robot was not actually facing the team's intended field forward direction.
- NavX sign or mounting assumptions are wrong, especially the upside-down mounting assumption in `Drivetrain.getRotation2d()`.
- A command is using `getPose()` after pose rotation has already been corrupted by a bad gyro reset.
- Vision is being blamed for pose jumps that actually started from an incorrect drivetrain heading baseline.