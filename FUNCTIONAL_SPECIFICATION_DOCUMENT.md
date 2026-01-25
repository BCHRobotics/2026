# FRC Team 2386 Robot Control System - Functional Specification Document

## Document Information
- **Document Version**: 1.2
- **Date**: January 24, 2026
- **Project**: FRC Team 2386 Robot Code (2026 Season)
- **Purpose**: Comprehensive specification for AI agents to understand and extend the robot control system

## 1. Project Overview

### 1.1 System Description
This is the robot control software for FRC Team 2386's competition robot, built using WPILib's command-based architecture. The system controls a swerve-drive robot with vision processing, actuator mechanisms, and autonomous path following capabilities.

### 1.2 Key Features
- **Swerve Drive System**: 4-module MAXSwerve drive with field-oriented control
- **Vision Processing**: Multi-camera AprilTag detection with pose estimation
- **Vision-Based Navigation**: Autonomous positioning relative to AprilTags
- **Diagnostic System**: Real-time robot position, AprilTag visibility, and motor current monitoring
- **Autonomous Operation**: PathPlanner-based autonomous routines
- **Real-time Telemetry**: SmartDashboard integration for monitoring and debugging
- **Example Subsystems**: Actuator and BallIntake subsystems (disabled, kept as development examples)

### 1.3 Competition Context
- **Season**: 2026 FRC Season
- **Team**: 2386 (BCHRobotics)
- **Framework**: WPILib 2026.2.1 Command-Based Architecture
- **Language**: Java 17

## 2. System Architecture

### 2.1 Core Architecture Pattern
The system follows WPILib's Command-Based Programming paradigm:
- **Robot.java**: Main robot class managing mode transitions
- **RobotContainer.java**: Dependency injection and command binding
- **Subsystems**: Hardware abstraction layers
- **Commands**: Discrete robot behaviors
- **Constants.java**: Configuration and tuning parameters

### 2.2 Package Structure
```
src/main/java/frc/
├── robot/
│   ├── Configs.java                 # WPILib configuration utilities for motor controllers for motor controllers
│   ├── Main.java                    # JVM entry point
│   ├── Robot.java                   # Main robot class
│   ├── RobotContainer.java          # Command binding and subsystem initialization
│   ├── Constants.java               # All configuration constants
│   ├── commands/                    # Command implementations
│   │   ├── actuator/                # Actuator control commands
│   │   ├── auto/                    # Autonomous commands
│   │   ├── ballintake/              # Ball intake control commands
│   │   ├── drive/                   # General drive commands
│   │   │   ├── GoToPositionCommand.java
│   │   │   └── GoToPositionRelativeCommand.java
│   │   ├── drivetrain/              # Drivetrain commands
│   │   └── vision/                  # Vision commands
│   │       ├── AlignToAprilTagCommand.java
│   │       └── GoToAprilTagCommand.java
│   └── subsystems/                  # Hardware subsystems
│       ├── Actuator.java            # SPARK MAX PID actuator
│       ├── Actuator2.java           # WPILib PID actuator
│       ├── BallIntake.java          # Ball intake subsystem
│       ├── Drivetrain.java          # Swerve drive subsystem
│       ├── MAXSwerveModule.java     # Individual swerve module
│       └── Vision.java              # PhotonVision integration
└── utils/                           # Utility classes
    ├── AutoUtils.java               # Autonomous helpers
    ├── DT/                          # Drive/navigation tools
    ├── M/                           # Math utilities
    ├── MathUtils.java               # General math functions
    └── SwerveUtils.java             # Swerve-specific utilities
```

## 3. Subsystem Specifications

### 3.1 Drivetrain Subsystem
**File**: `Drivetrain.java`
**Purpose**: Controls the 4-module swerve drive system

**Key Responsibilities**:
- Field-oriented drive control with slew rate limiting
- Odometry tracking using wheel encoders and IMU
- Integration with vision-based pose estimation
- Autonomous path following via PathPlanner

**Key Methods**:
- `drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit)`: Manual drive control with rate limiting option
- `setModuleStates(SwerveModuleState[] desiredStates)`: Direct module control
- `resetOdometry(Pose2d pose)`: Reset position tracking
- `getPose()`: Get current robot pose
- `addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3,N1> stdDevs)`: Vision fusion
- `setVision(Vision vision)`: Connect Vision subsystem for diagnostics
- `printDiagnostics()`: Output robot position, AprilTag info, and motor currents (once per second)

**Configuration Constants** (DriveConstants):
- Wheelbase and track width dimensions
- Maximum speeds and accelerations
- PID gains for drive and turn control
- CAN IDs for all motors

### 3.2 Vision Subsystem
**File**: `Vision.java`
**Purpose**: Multi-camera AprilTag detection and pose estimation

**Key Responsibilities**:
- PhotonVision camera management (up to 4 cameras)
- AprilTag pose estimation with multi-tag tracking
- Kalman filter fusion with wheel odometry
- Dynamic trust adjustment based on distance/tag count

**Key Methods**:
- `getCamera(int index)`: Get specific camera instance
- `getCameraModules()`: Get list of all camera modules
- `getActiveCameraCount()`: Get number of active cameras
- `getFieldLayout()`: Get AprilTag field layout
- `getVisibleAprilTags()`: Get list of currently visible AprilTag IDs
- `getDetailedAprilTagInfo()`: Get detailed info including ID, ambiguity, distance, camera, and pose update status
- `AprilTagInfo` class: Contains tag ID, ambiguity, update status, camera name, and distance

**Configuration Constants** (VisionConstants):
- Camera names and enable/disable flags
- Robot-to-camera transforms
- Pose estimation standard deviations
- Multi-tag tracking parameters

### 3.3 Actuator Subsystem (DISABLED - Example Code)
**File**: `Actuator.java`
**Purpose**: SPARK MAX onboard PID control for mechanisms
**Status**: ⚠️ DISABLED - Hardware does not physically exist, kept as example code

**Key Responsibilities**:
- Closed-loop position control using NEO motor
- Soft limits and current limiting
- Manual override capability
- Preset position management

**Key Methods**:
- `setPosition(double position)`: Set target position with PID control
- `setManualSpeed(double speed)`: Direct speed control (bypasses PID)
- `stop()`: Stop motor immediately
- `getPosition()`: Get current encoder position
- `getVelocity()`: Get current velocity
- `atSetpoint()`: Check if at target position
- `resetPosition()`: Reset encoder position

**Configuration Constants** (ActuatorConstants):
- CAN ID and motor configuration
- PID gains and feedforward terms
- Position limits and preset positions
- Current limits and soft stops

### 3.4 Actuator2 Subsystem (DISABLED - Example Code)
**File**: `Actuator2.java`
**Purpose**: WPILib PID control for mechanisms
**Status**: ⚠️ DISABLED - Hardware does not physically exist, kept as example code

**Key Responsibilities**:
- RoboRIO-based PID control (alternative to SPARK MAX PID)
- Real-time tuning via SmartDashboard
- Similar functionality to Actuator but more flexible

**Key Methods**: (Similar to Actuator)
- `setPosition(double position)`
- `setSpeed(double speed)`
- `resetEncoder()`
- `getPosition()`

**Configuration Constants** (Actuator2Constants):
- CAN ID and motor configuration
- PID gains (tunable via dashboard)
- Position limits and presets

### 3.5 Ball Intake Subsystem (DISABLED - Example Code)
**File**: `BallIntake.java`
**Purpose**: Variable speed motor control for collecting and managing game pieces
**Status**: ⚠️ DISABLED - Hardware does not physically exist, kept as example code

**Key Responsibilities**:
- Variable speed motor control for ball intake/ejection
- Configurable speeds for different operating modes
- Current limiting and motor protection
- Real-time telemetry and status monitoring

**Key Methods**:
- `setSpeed(double speed)`: Set motor speed with deadband filtering
- `intake()`: Start intaking balls at configured speed
- `eject()`: Eject balls at configured reverse speed
- `hold()`: Hold balls with low speed
- `stop()`: Stop motor completely
- `getSpeed()`: Get current speed setting
- `getCurrent()`: Get motor current draw
- `isRunning()`: Check if intake is active

**Configuration Constants** (BallIntakeConstants):
- CAN ID and motor configuration
- Intake, eject, and hold speeds
- Current limits and deadband settings

## 4. Command Specifications

### 4.1 Drive Commands
**TeleopDriveCommand**: Main teleop driving
- Processes PS5 controller input
- Applies deadbands and slew rate limiting
- Field-oriented control with optional rate limiting

**AlignToAprilTagCommand**: Vision-assisted alignment
- Uses vision system to align to AprilTags
- PID control for precise positioning
- Distance and angle correction

**GoToAprilTagCommand**: Autonomous navigation to AprilTag
- Navigates robot to specified distance in front of AprilTag
- Field-relative positioning with rotation control
- Configurable target tag ID and distance
- PID-based control with position and rotation tolerances
- Robot faces the AprilTag with camera/front side

**GoToPositionCommand**: Autonomous navigation to field position
- Navigates robot to any X, Y coordinate on the field
- Field-relative positioning with configurable rotation
- Inputs: X (meters), Y (meters), Rotation (degrees)
- PID-based control with position and rotation tolerances
- Uses odometry for field-relative navigation
- Can be constructed with individual parameters or a Pose2d object

**GoToPositionRelativeCommand**: Alliance-relative autonomous navigation
- Navigates robot using alliance-relative coordinates
- Automatically mirrors coordinates for red alliance (opposite diagonal corner)
- Inputs: X (meters), Y (meters), Rotation (degrees), Alliance (Red/Blue)
- For Blue Alliance: (0,0) is at blue corner (standard FRC coordinates)
- For Red Alliance: (0,0) is mirrored to opposite diagonal corner
  - X coordinates: red_x = FIELD_LENGTH - blue_x (16.54m field)
  - Y coordinates: red_y = FIELD_WIDTH - blue_y (8.21m field)
  - Rotation: red_rotation = 180° - blue_rotation
- Enables reusable autonomous routines regardless of alliance color
- Can auto-detect alliance from DriverStation or accept explicit Alliance parameter
- PID-based control with position and rotation tolerances

### 4.2 Actuator Commands (DISABLED - Example Code)
**Status**: ⚠️ All actuator commands disabled as hardware does not exist

**Position Commands**: Move to preset positions
- Home, mid, and max extension positions
- Smooth transitions with PID control

**Manual Commands**: Direct control
- Joystick input with scaling
- Override capability for troubleshooting

### 4.3 Autonomous Commands
**Path Following**: PathPlanner integration
- Load and execute pre-planned paths
- Real-time path generation capability
- Event markers for coordinated actions

### 4.4 Ball Intake Commands (DISABLED - Example Code)
**Status**: ⚠️ All ball intake commands disabled as hardware does not exist

**IntakeCommand**: Start ball collection
- Runs intake motor at configured intake speed
- Continues until interrupted or stopped

**EjectCommand**: Expel balls
- Runs intake motor in reverse at configured eject speed
- Continues until interrupted or stopped

**HoldCommand**: Retain balls
- Runs intake motor at low hold speed
- Prevents balls from falling out without collecting more

**StopCommand**: Immediate stop
- Instantly stops the intake motor
- Instantaneous command completion

## 5. Utility Classes

### 5.1 Math Utilities
**MathUtils.java**: General mathematical functions
- Vector operations (2D/3D/4D)
- Polygon operations
- Interpolation and filtering

**SwerveUtils.java**: Swerve-specific calculations
- Module state optimization
- Kinematics transformations
- Motion profiling

### 5.2 Navigation Tools (DT Package)
**Purpose**: Field positioning and visualization
- Robot pose tracking
- Path visualization
- Field coordinate transformations

### 5.3 Autonomous Utilities
**AutoUtils.java**: Autonomous helper functions
- Path loading and validation
- Event timing coordination
- State machine management

## 6. Hardware Configuration

### 6.1 Motor Controllers
| Component | Type | CAN ID | Notes |
|-----------|------|--------|-------|
| FL Drive | NEO (SPARK MAX) | 12 | MAXSwerve Module |
| FL Turn | NEO (SPARK MAX) | 13 | MAXSwerve Module |
| FR Drive | NEO (SPARK MAX) | 14 | MAXSwerve Module |
| FR Turn | NEO (SPARK MAX) | 15 | MAXSwerve Module |
| RL Drive | NEO (SPARK MAX) | 10 | MAXSwerve Module |
| RL Turn | NEO (SPARK MAX) | 11 | MAXSwerve Module |
| RR Drive | NEO (SPARK MAX) | 16 | MAXSwerve Module |
| RR Turn | NEO (SPARK MAX) | 17 | MAXSwerve Module |
| ~~Actuator~~ | ~~NEO (SPARK MAX)~~ | ~~20~~ | ⚠️ DISABLED - Example code only |
| ~~Actuator2~~ | ~~NEO (SPARK MAX)~~ | ~~21~~ | ⚠️ DISABLED - Example code only |
| ~~Ball Intake~~ | ~~NEO (SPARK MAX)~~ | ~~22~~ | ⚠️ DISABLED - Example code only |

### 6.2 Sensors
| Component | Type | Interface | Notes |
|-----------|------|-----------|-------|
| Gyroscope | NavX | SPI | Robot heading |
| Cameras | PhotonVision | Network | AprilTag detection |

### 6.3 Mechanical Specifications
- **Wheelbase**: 24.5 inches
- **Track Width**: 24.5 inches
- **Wheel Diameter**: 3 inches
- **Drive Ratio**: MAXSwerve 12T pinion

## 7. Software Dependencies

### 7.1 Core Framework
- **WPILib**: 2026.2.1 (GradleRIO plugin)
- **Java**: 17 (source and target compatibility)
- **Gradle**: 8.11 (build system)

### 7.2 Vendor Libraries
- **REVLib**: SPARK MAX motor controllers (2026.0.0)
- **PhotonLib**: Vision processing (v2026.0.1-beta)
- **PathPlannerLib**: Autonomous path planning (2026.1.2)
- **Studica**: NavX IMU (2026.0.0)

## 8. Control Interface

### 8.1 Driver Controls (PS5 Controller - Port 0)
**Drive**:
- Left Stick: Translation (X/Y) - Forward/backward and strafe
- Right Stick X: Rotation (Z)
- Field-oriented with slew rate limiting

**Active Vision Controls**:
- **Square Button**: Navigate to AprilTag 12 at 1.0m distance (autonomous positioning)
- **Options Button**: Align to AprilTag 4 (while held, 5s timeout)

**Disabled Controls (Example Code)**:
⚠️ The following controls are disabled as the hardware does not exist:

~~**Actuator 1 (SPARK MAX PID)**~~:
- ~~Cross Button: Home position (retracted)~~
- ~~Circle Button: Mid position~~
- ~~Triangle Button: Max position (extended)~~
- ~~Right Stick Y: Manual control~~

~~**Actuator 2 (WPILib PID)**~~:
- ~~L2/R2 Triggers: Manual control (extend/retract while held)~~
- ~~D-Pad Down: Home position~~
- ~~D-Pad Left/Right: Mid position~~
- ~~D-Pad Up: Max position~~
- ~~Create Button: Reset encoder~~

~~**Ball Intake**~~:
- ~~L1 Button: Intake balls (while held)~~
- ~~R1 Button: Eject balls (while held)~~
- ~~Touchpad: Hold balls (while held)~~
- ~~PS Button: Stop intake (momentary)~~

### 8.2 Autonomous Configuration
- SendableChooser for autonomous selection
- SmartDashboard for real-time monitoring
- PathPlanner GUI for path creation

## 9. Development Guidelines for AI Agents

### 9.1 Code Organization Principles
1. **Single Responsibility**: Each class/subsystem has one clear purpose
2. **Dependency Injection**: All subsystems instantiated in RobotContainer
3. **Constants Centralization**: All configuration in Constants.java
4. **Command Composition**: Complex behaviors built from simple commands

### 9.2 Extension Points for New Functionality

#### Adding New Subsystems
1. Create subsystem class in `subsystems/` package
2. Add instantiation in RobotContainer
3. Add constants section in Constants.java
4. Create associated commands in appropriate `commands/` subdirectory

#### Adding New Commands
1. Extend CommandBase or implement Command interface
2. Follow naming convention: `<Action><Subsystem>Command`
3. Add to appropriate package (actuator/, drive/, vision/, etc.)
4. Bind to controls in RobotContainer

#### Adding New Utilities
1. Add to `utils/` package with clear naming
2. Document public API with JavaDoc
3. Include unit tests if applicable
4. Update this FSD when adding new utilities

### 9.3 Configuration Management
- **Constants.java**: All tunable parameters
- **VisionConstants**: Camera configurations
- **DriveConstants**: Motor and kinematics settings
- **ActuatorConstants**: Mechanism tuning parameters

### 9.4 Testing and Validation
- **Unit Tests**: Individual component testing
- **Integration Tests**: Subsystem interaction validation
- **Simulation**: WPILib simulation environment
- **Hardware Testing**: Real robot validation

## 10. Future Enhancement Specifications

### 10.1 Potential New Features
1. **Advanced Vision**: Multi-target tracking, object detection
2. **Path Optimization**: Real-time path replanning
3. **Mechanism Control**: Additional actuators, coordinated motion
4. **Telemetry**: Enhanced data logging and analysis
5. **Autonomous**: Complex multi-step routines

### 10.2 AI Agent Implementation Guidelines
When implementing new features:

1. **Follow Existing Patterns**: Maintain consistency with current architecture
2. **Document Changes**: Update this FSD with new specifications
3. **Test Thoroughly**: Validate on simulation before hardware testing
4. **Consider Performance**: Real-time constraints for robot control
5. **Maintain Modularity**: Keep subsystems independent and reusable

### 10.3 Integration Requirements
- **Thread Safety**: Ensure thread-safe operation in multi-threaded environment
- **Real-time Performance**: Meet 20ms periodic execution requirements
- **Resource Management**: Efficient use of network, CPU, and memory
- **Error Handling**: Graceful degradation on sensor/motor failures

---

## Appendix A: File Structure Reference

```
2025_Updated/
├── build.gradle                 # Gradle build configuration
├── settings.gradle              # Project settings
├── README.md                    # Project documentation
├── vendordeps/                  # Vendor library definitions
│   ├── PhotonLib.json
│   ├── PathPlannerLib.json
│   ├── REVLib.json
│   └── Studica.json
├── src/main/java/frc/
│   ├── robot/
│   │   ├── Main.java
│   │   ├── Robot.java
│   │   ├── RobotContainer.java
│   │   ├── Constants.java
│   │   ├── commands/
│   │   └── subsystems/
│   └── utils/
├── src/main/deploy/             # Files deployed to robot
│   └── pathplanner/            # Autonomous paths and settings
└── .wpilib/                     # WPILib project configuration
```

## Appendix B: Key Interfaces and APIs

### Command-Based Architecture
- `Subsystem`: Hardware abstraction interface
- `Command`: Executable robot behavior
- `CommandScheduler`: Manages command execution
- `SendableChooser`: Autonomous selection

### WPILib Core APIs
- `SwerveDriveKinematics`: Swerve math
- `SwerveDrivePoseEstimator`: Sensor fusion
- `PIDController`: Control loops
- `PhotonPoseEstimator`: Vision processing

### Vendor APIs
- `CANSparkMax`: Motor control
- `PhotonCamera`: Vision processing
- `PathPlanner`: Path following
- `AHRS`: IMU interface

---

*This document serves as the comprehensive specification for the FRC Team 2386 robot control system. AI agents should reference this document when implementing new features or modifying existing functionality to ensure consistency with the established architecture and patterns.*