# Logged & Dashboard Values

All values published to AdvantageKit Logger or SmartDashboard.  
**[Y]** = currently being published | **[N]** = exists in code but commented out

---

## AdvantageKit Logger (`Logger.recordOutput`)

### Drivetrain

| Key | Type | Published |
|-----|------|-----------|
| `Drivetrain/Pose` | Pose2d | [Y] |
| `Drivetrain/Pose3d` | Pose3d | [Y] |
| `Drivetrain/OdometryPose` | Pose2d | [Y] |
| `Drivetrain/OdometryPose3d` | Pose3d | [Y] |
| `Drivetrain/ModuleStates` | SwerveModuleState[] | [Y] |
| `Drivetrain/HeadingDegrees` | double | [Y] |
| `Drivetrain/ChassisSpeeds/VxMetersPerSecond` | double | [Y] |
| `Drivetrain/ChassisSpeeds/VyMetersPerSecond` | double | [Y] |
| `Drivetrain/ChassisSpeeds/OmegaRadiansPerSecond` | double | [Y] |
| `Drivetrain/VisionPoseErrorMeters` | double | [Y] |

### Vision

| Key | Type | Published |
|-----|------|-----------|
| `Vision/RobotPoses` | Pose3d[] | [Y] |
| `Vision/CameraPoses` | Pose3d[] | [Y] |
| `Vision/TagPoses` | Pose3d[] | [Y] |
| `Vision/VisibleTagCount` | int | [Y] |
| `Vision/VisibleTagIds` | int[] | [Y] |
| `Vision/HasVisionPose` | boolean | [Y] |
| `Vision/LastAcceptedTagCount` | int | [Y] |
| `Vision/LastAcceptedAvgTagDistanceMeters` | double | [Y] |
| `Vision/{cameraName}/HasTargets` | boolean | [Y] |
| `Vision/{cameraName}/LastRejectReason` | String | [Y] |
| `Vision/{cameraName}/TotalRejectedFrames` | int | [Y] |
| `Vision/{cameraName}/LatencyMillis` | double | [Y] |
| `Vision/{cameraName}/FrameTimestamp` | double | [Y] |
| `Vision/{cameraName}/AppliedXYStdDev` | double | [Y] |
| `Vision/{cameraName}/AppliedThetaStdDev` | double | [Y] |

### Auto (PathPlanner callbacks)

| Key | Type | Published |
|-----|------|-----------|
| `Auto/ActivePath` | Pose2d[] | [Y] |
| `Auto/TargetPose` | Pose2d | [Y] |
| `Auto/TargetPose3d` | Pose3d | [Y] |

### BallIntake

| Key | Type | Published |
|-----|------|-----------|
| `BallIntake/CalibrateState` | String | [Y] |
| `BallIntake/ExtendCurrent` | double | [Y] |
| `BallIntake/FilteredExtendCurrent` | double | [Y] |
| `BallIntake/CurrentSpike` | boolean | [Y] |
| `BallIntake/ExtendPosition` | double | [Y] |
| `BallIntake/TargetExtendPosition` | double | [Y] |
| `BallIntake/ExtendEnabled` | boolean | [Y] |
| `BallIntake/JiggleActive` | boolean | [Y] |
| `BallIntake/RunEnabled` | boolean | [Y] |
| `BallIntake/AtTarget` | boolean | [Y] |
| `BallIntake/RunCurrent` | double | [Y] |
| `BallIntake/ExtendAppliedOutput` | double | [Y] |
| `BallIntake/RunAppliedOutput` | double | [Y] |

### Climber

| Key | Type | Published |
|-----|------|-----------|
| `Climber/ForwardLimitPressed` | boolean | [Y] |
| `Climber/ReverseLimitPressed` | boolean | [Y] |
| `Climber/LimitPressed` | boolean | [Y] |
| `Climber/CurrentSpikeDetected` | boolean | [Y] |
| `Climber/PlateDetected` | boolean | [Y] |
| `Climber/ExtendLimitReached` | boolean | [Y] |
| `Climber/RetractLimitReached` | boolean | [Y] |
| `Climber/MotionTimedOut` | boolean | [Y] |
| `Climber/MotionState` | String | [Y] |
| `Climber/Current` | double | [Y] |
| `Climber/FilteredCurrent` | double | [Y] |
| `Climber/AppliedOutput` | double | [Y] |

### Shooter

| Key | Type | Published |
|-----|------|-----------|
| `Shooter/Distance` | double | [Y] |
| `Shooter/TargetRPM` | double | [Y] |
| `Shooter/ReadyRPM` | double | [Y] |
| `Shooter/Shooter1RPM` | double | [Y] |
| `Shooter/Shooter2RPM` | double | [Y] |
| `Shooter/AppliedFeederSpeed` | double | [Y] |
| `Shooter/VortexSpeedShotActive` | boolean | [Y] |
| `Shooter/RPM` | double | [Y] |
| `Shooter/Charged` | boolean | [Y] |
| `Shooter/Active` | boolean | [Y] |
| `Shooter/Motor1CurrentAmps` | double | [Y] |
| `Shooter/Motor2CurrentAmps` | double | [Y] |
| `Shooter/FeederCurrentAmps` | double | [Y] |
| `Shooter/RPMError` | double | [Y] |

### ClimbCommand

| Key | Type | Published |
|-----|------|-----------|
| `ClimbCommand/Running` | boolean | [Y] |
| `ClimbCommand/Phase` | String | [Y] |
| `ClimbCommand/LastResult` | String | [Y] |
| `ClimbCommand/InitialPose` | Pose2d | [Y] |
| `ClimbCommand/SelectedStartPose` | Pose2d | [Y] |
| `ClimbCommand/TargetPose` | Pose2d | [Y] |
| `ClimbCommand/AtStartPose` | boolean | [Y] |
| `ClimbCommand/CurrentPoseX` | double | [Y] |
| `ClimbCommand/CurrentPoseY` | double | [Y] |
| `ClimbCommand/CurrentHeadingDegrees` | double | [Y] |

### Vision (additional — per camera & tuning)

| Key | Type | Published |
|-----|------|-----------|
| `Vision/{cameraName}/PoseDeltaMeters` | double | [Y] |
| `Vision/{cameraName}/RotationDeltaDegrees` | double | [Y] |
| `Vision/{cameraName}/TotalRejectedFrames` | int | [Y] |
| `Vision/{cameraName}/LatencyMillis` | double | [Y] |
| `Vision/{cameraName}/FrameTimestamp` | double | [Y] |
| `Vision/{cameraName}/AppliedXYStdDev` | double | [Y] |
| `Vision/{cameraName}/AppliedThetaStdDev` | double | [Y] |
| `Vision/Tuning/MaxAmbiguity` | double | [Y] |
| `Vision/Tuning/SingleTagXYStdDev` | double | [Y] |
| `Vision/Tuning/SingleTagThetaStdDev` | double | [Y] |
| `Vision/Tuning/MultiTagXYStdDev` | double | [Y] |
| `Vision/Tuning/MultiTagThetaStdDev` | double | [Y] |
| `Vision/Tuning/DistanceWeight` | double | [Y] |
| `Vision/Tuning/RotationDistanceWeight` | double | [Y] |
| `Vision/Tuning/MaxSingleTagDistance` | double | [Y] |
| `Vision/Tuning/MaxMultiTagDistance` | double | [Y] |
| `Vision/Tuning/MaxSingleTagPoseDeltaMeters` | double | [Y] |
| `Vision/Tuning/MaxMultiTagPoseDeltaMeters` | double | [Y] |
| `Vision/Tuning/MaxSingleTagRotationDeltaDegrees` | double | [Y] |
| `Vision/Tuning/MaxMultiTagRotationDeltaDegrees` | double | [Y] |

---

## SmartDashboard

### Drivetrain

| Key | Type | Published |
|-----|------|-----------|
| `Gyro Heading` | double | [Y] |

### Vision

| Key | Type | Published |
|-----|------|-----------|
| `Vision/Field` | Field2d | [Y] |
| `Vision/HasVisionPose` | boolean | [Y] |
| `Vision/{cameraName}/LastRejectReason` | String | [Y] |
| `Vision/{cameraName}/PoseDeltaMeters` | double | [Y] |
| `Vision/{cameraName}/RotationDeltaDegrees` | double | [Y] |
| `Vision/LastAcceptedCamera` | String | [N] |
| `Vision/LastAcceptedTagCount` | double | [N] |
| `Vision/LastAcceptedAvgDistance` | double | [N] |
| `Vision/LastAcceptedXYStdDev` | double | [N] |
| `Vision/LastAcceptedThetaStdDev` | double | [N] |
| `Vision/EstimatedPose/X` | double | [N] |
| `Vision/EstimatedPose/Y` | double | [N] |
| `Vision/EstimatedPose/Rotation` | double | [N] |
| `Vision/PoseTimestamp` | double | [N] |

### Vision Tuning Inputs *(SmartDashboard read keys — uncomment `publishTuningDefaults()` lines to also publish)*

| Key | Type | Published |
|-----|------|-----------|
| `Vision/Tuning/MaxAmbiguity` | double | [Y] |
| `Vision/Tuning/SingleTagXYStdDev` | double | [Y] |
| `Vision/Tuning/SingleTagThetaStdDev` | double | [Y] |
| `Vision/Tuning/MultiTagXYStdDev` | double | [Y] |
| `Vision/Tuning/MultiTagThetaStdDev` | double | [Y] |
| `Vision/Tuning/DistanceWeight` | double | [Y] |
| `Vision/Tuning/RotationDistanceWeight` | double | [Y] |
| `Vision/Tuning/MaxSingleTagDistance` | double | [Y] |
| `Vision/Tuning/MaxMultiTagDistance` | double | [Y] |
| `Vision/Tuning/MaxSingleTagPoseDeltaMeters` | double | [Y] |
| `Vision/Tuning/MaxMultiTagPoseDeltaMeters` | double | [Y] |
| `Vision/Tuning/MaxSingleTagRotationDeltaDegrees` | double | [Y] |
| `Vision/Tuning/MaxMultiTagRotationDeltaDegrees` | double | [Y] |

### BallIntake

| Key | Type | Published |
|-----|------|-----------|
| `BallIntake/CalibrateState` | String | [Y] |
| `BallIntake/ExtendCurrent` | double | [Y] |
| `BallIntake/FilteredExtendCurrent` | double | [Y] |
| `BallIntake/current spike` | boolean | [Y] |
| `BallIntake/ExtendPosition` | double | [Y] |
| `BallIntake/TargetExtendPosition` | double | [Y] |
| `BallIntake/ExtendEnabled` | boolean | [Y] |
| `BallIntake/AtTarget` | boolean | [Y] |
| `BallIntake/RunCurrent` | double | [Y] |
| `BallIntake/JiggleActive` | boolean | [Y] |
| `BallIntake/RunEnabled` | boolean | [Y] |

### Climber

| Key | Type | Published |
|-----|------|-----------|
| `Climber/ForwardLimitPressed` | boolean | [Y] |
| `Climber/ReverseLimitPressed` | boolean | [Y] |
| `Climber/LimitPressed` | boolean | [Y] |
| `Climber/CurrentSpikeDetected` | boolean | [Y] |
| `Climber/PlateDetected` | boolean | [Y] |
| `Climber/ExtendLimitReached` | boolean | [Y] |
| `Climber/RetractLimitReached` | boolean | [Y] |
| `Climber/MotionTimedOut` | boolean | [Y] |
| `Climber/MotionState` | String | [Y] |
| `Climber/Current` | double | [Y] |
| `Climber/FilteredCurrent` | double | [Y] |
| `Climber/AppliedOutput` | double | [Y] |

### Shooter

| Key | Type | Published |
|-----|------|-----------|
| `Shooter/Distance` | double | [Y] |
| `Shooter/TargetRPM` | double | [Y] |
| `Shooter/ReadyRPM` | double | [Y] |
| `Shooter/Shooter1RPM` | double | [Y] |
| `Shooter/Shooter2RPM` | double | [Y] |
| `Shooter/AppliedFeederSpeed` | double | [Y] |
| `Shooter/VortexSpeedShotActive` | boolean | [Y] |
| `Shooter/RPM` | double | [Y] |
| `Shooter/Charged` | boolean | [Y] |
| `Shooter/Active` | boolean | [Y] |
| `Shooter/VortexSpeedShotReady` | boolean | [N] |
| `Shooter/Motor1/F` | double | [N] |
| `Shooter/Motor1/P` | double | [N] |
| `Shooter/Motor1/I` | double | [N] |
| `Shooter/Motor1/D` | double | [N] |
| `Shooter/Motor2/F` | double | [N] |
| `Shooter/Motor2/P` | double | [N] |
| `Shooter/Motor2/I` | double | [N] |
| `Shooter/Motor2/D` | double | [N] |
| `Shooter/MaxOutput` | double | [N] |

### ClimbCommand

| Key | Type | Published |
|-----|------|-----------|
| `ClimbCommand/Running` | boolean | [Y] |
| `ClimbCommand/Phase` | String | [Y] |
| `ClimbCommand/LastResult` | String | [Y] |
| `ClimbCommand/CurrentPoseX` | double | [Y] |
| `ClimbCommand/CurrentPoseY` | double | [Y] |
| `ClimbCommand/CurrentHeadingDegrees` | double | [Y] |
| `ClimbCommand/AtStartPose` | boolean | [Y] |
| `ClimbCommand/InitialPose` | String | [Y] |
| `ClimbCommand/SelectedStartPose` | String | [Y] |
| `ClimbCommand/TargetPose` | String | [Y] |

### VisionTuningPath

| Key | Type | Published |
|-----|------|-----------|
| `VisionTuningPath/Running` | boolean | [Y] |
| `VisionTuningPath/LastResult` | String | [Y] |
| `VisionTuningPath/TargetIndex` | double | [Y] |
| `VisionTuningPath/MaxSpeedMetersPerSecond` | double | [N] |
| `VisionTuningPath/PathPoints` | String | [N] |
| `VisionTuningPath/HeldHeadingDegrees` | double | [N] |
| `VisionTuningPath/CurrentTargetX` | double | [N] |
| `VisionTuningPath/CurrentTargetY` | double | [N] |
| `VisionTuningPath/CurrentPoseX` | double | [N] |
| `VisionTuningPath/CurrentPoseY` | double | [N] |
| `VisionTuningPath/CurrentHeadingDegrees` | double | [N] |
| `VisionTuningPath/RequestedXSpeed` | double | [N] |
| `VisionTuningPath/RequestedYSpeed` | double | [N] |
| `VisionTuningPath/RequestedRotationSpeed` | double | [N] |

### Dashboard Widgets / Choosers

| Key | Type | Published |
|-----|------|-----------|
| `Auto Mode` | SendableChooser | [Y] |
| `Climb Start Pose` | SendableChooser | [Y] |
| `Intake Extend Override` | SendableChooser | [Y] |
| `Climb Command` | Sendable (Command) | [Y] |
| `VisionTuningPath` | Sendable (Command) | [Y] |
