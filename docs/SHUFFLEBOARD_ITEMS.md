# Shuffleboard / SmartDashboard Elements

This file contains a list of all elements currently being published to Shuffleboard/SmartDashboard.
`Y` means the value is currently published by the codebase.
`N` means the value is not currently published by the codebase.

## RobotContainer
- [Y] `Auto Mode` (SendableChooser)
- [N] `Field Pose` (SendableChooser)
- [N] `PP Translation PID` (SendableChooser)
- [N] `PP Rotation PID` (SendableChooser)
- [N] `Autonomous Mode` (SendableChooser) - *Duplicate/not present; `Auto Mode` is the published chooser*
- [Y] `Climb Start Pose` (SendableChooser)

## Drivetrain
- [Y] `Gyro Heading` (Number)
- [Y] `Field/Fused` (Field2d)
- [Y] `Field/Odometry` (Field2d)
- [Y] `Field/Vision` (Field2d)
- [Y] `Pose/Fused/X` (Number)
- [Y] `Pose/Fused/Y` (Number)
- [Y] `Pose/Fused/HeadingDegrees` (Number)
- [Y] `Pose/Odometry/X` (Number)
- [Y] `Pose/Odometry/Y` (Number)
- [Y] `Pose/Odometry/HeadingDegrees` (Number)
- [Y] `Pose/Vision/Available` (Boolean)
- [Y] `Pose/Vision/X` (Number)
- [Y] `Pose/Vision/Y` (Number)
- [Y] `Pose/Vision/HeadingDegrees` (Number)
- [Y] `Pose/Vision/Timestamp` (Number)
- [Y] `CAN/DrivetrainStatus` (String)
- [Y] `Diagnostics/Drivetrain` (String)

## Ball Intake
- [Y] `BallIntake/CalibrateState` (String)
- [Y] `BallIntake/ExtendCurrent` (Number)
- [Y] `BallIntake/FilteredExtendCurrent` (Number)
- [Y] `BallIntake/ExtendPosition` (Number)
- [Y] `BallIntake/TargetExtendPosition` (Number)
- [Y] `BallIntake/ExtendEnabled` (Boolean)
- [Y] `BallIntake/JiggleActive` (Boolean)
- [Y] `BallIntake/RunEnabled` (Boolean)
- [Y] `BallIntake/AtTarget` (Boolean)
- [Y] `BallIntake/RunCurrent` (Number)
- [Y] `BallIntake/Speed` (Number)
- [Y] `BallIntake/Current` (Number)
- [Y] `BallIntake/Running` (Boolean)

## Vision
- [N] `Vision Field` (Field2d)
- [N] `Vision/Total Cameras` (Number)
- [N] `Vision/Cameras With Targets` (Number)
- [N] `Vision/Total Targets` (Number)

### Per Camera (Vision/Cam{index}/...)
- [Y] `Vision/<cameraName>/HasTargets` (Boolean)
- [Y] `Vision/<cameraName>/TagCount` (Number)
- [Y] `Vision/<cameraName>/IsMultiTag` (Boolean)
- [Y] `Vision/<cameraName>/DetectionStatus` (String: `NO_DATA`, `NO_TAGS`, `SINGLE_TAG`, `MULTI_TAG`)
- [N] `Vision/<cameraName>/Timestamp` (Number)
- [N] `Vision/<cameraName>/Name` (String)
- [N] `Vision/<cameraName>/Best Target ID` (Number)
- [N] `Vision/<cameraName>/Ambiguity` (Number)
- [N] `Vision/<cameraName>/Area` (Number)
- [N] `Vision/<cameraName>/Alliance` (String)

## Vision WebServer
- [N] `WebServer/Running` (Boolean)
- [N] `WebServer/Port` (Number)

## Suggested Additions
- `Vision/Total Cameras` (Number) - publish `cameraModules.size()` for quick health checks.
- `Vision/Cameras With Targets` (Number) - useful summary for operator awareness.
- `Vision/Total Targets` (Number) - quick measure of overall AprilTag visibility.
- `Vision/<cameraName>/LastUpdateSeconds` (Number) - helps distinguish disconnects from just seeing no tags.
- `Vision/<cameraName>/LastRejectReason` (String) - useful when detections are present but pose updates are rejected.
- `CAN/RobotStatus` (String) - aggregate drivetrain, shooter, intake, and climber CAN health into one summary.
