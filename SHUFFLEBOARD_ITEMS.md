# Shuffleboard / SmartDashboard Elements

This file contains a list of all elements currently being published to Shuffleboard/SmartDashboard.
Please edit this file to indicate which items should be kept or removed.

## RobotContainer
- [ ] `Auto Mode` (SendableChooser)
- [ ] `Field Pose` (SendableChooser)
- [R] `PP Translation PID` (SendableChooser)
- [R] `PP Rotation PID` (SendableChooser)
- [ ] `Autonomous Mode` (SendableChooser) - *Note: Appears to be a duplicate of "Auto Mode"*

## Drivetrain
- [ ] `Gyro Heading` (Number)

## Ball Intake
- [R] `Ball Intake/Speed` (Number)
- [R] `Ball Intake/Current` (Number)
- [R] `Ball Intake/Running` (Boolean)

## Vision
- [ ] `Vision Field` (Field2d)
- [R] `Vision/Total Cameras` (Number)
- [R] `Vision/Cameras With Targets` (Number)
- [R] `Vision/Total Targets` (Number)

### Per Camera (Vision/Cam{index}/...)
- [R] `.../Has Targets` (Boolean)
- [R] `.../Target Count` (Number)
- [R] `.../Timestamp` (Number)
- [R] `.../Name` (String)
- [R] `.../Best Target ID` (Number)
- [R] `.../Ambiguity` (Number)
- [R] `.../Area` (Number)
- [R] `.../Alliance` (String)

## Vision WebServer
- [R] `WebServer/Running` (Boolean)
- [R] `WebServer/Port` (Number)
