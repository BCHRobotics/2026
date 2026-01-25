# FRC Team 2386 - 2026 REBUILT Robot Code

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![Java](https://img.shields.io/badge/Java-17-blue)]()
[![WPILib](https://img.shields.io/badge/WPILib-2026-orange)]()

Official robot code for **FRC Team 2386** competing in the 2026 FIRST Robotics Competition season - **REBUILT**.

##  Robot Features

### Drive System
- **MAXSwerve Drive**: Four-module swerve drive with NEO motors
- **Field-Oriented Control**: Precise navigation with NavX gyroscope
- **Vision-Assisted Odometry**: Kalman filter fusion with AprilTag vision
- **PathPlanner Integration**: Autonomous path following with on-the-fly generation
- **Default Teleop Command**: Dual-stick control with slew rate limiting for smooth driving
- **Diagnostic System**: Real-time position, AprilTag visibility, and motor current monitoring (once per second)

### Vision System
- **Multi-Camera PhotonVision**: Support for up to 4 cameras with individual enable/disable
- **Real-Time AprilTag Detection**: Simultaneous processing from all active cameras
- **Multi-Tag Tracking**: Enhanced accuracy when multiple tags are visible
- **Unscented Kalman Filter**: Optimal fusion of wheel odometry and vision measurements
- **Dynamic Trust Adjustment**: Adapts confidence based on distance and tag count
- **Parallel Pose Fusion**: Aggregates measurements from all cameras for maximum accuracy
- **Vision Alignment Commands**: Automated AprilTag alignment for precise positioning
- **Vision-Based Navigation**: GoToAprilTagCommand for autonomous positioning relative to AprilTags
- **Field Position Navigation**: GoToPositionCommand for autonomous navigation to any field location
- **Alliance-Relative Navigation**: GoToPositionRelativeCommand with automatic coordinate mirroring for red/blue alliances
- **Ambiguity Filtering**: Rejects low-quality pose estimates (>0.2 ambiguity) to ensure accuracy
- **Detailed Diagnostics**: Real-time reporting of visible tags with confidence, distance, and update status

### Actuator System (DISABLED - Example Code)
⚠️ **Note**: The following actuator subsystems are disabled as the hardware does not physically exist on this robot. The code is kept as examples for future development.

- **Dual PID Examples**: SPARK MAX onboard PID (Actuator) and WPILib PID (Actuator2)
- **PID Position Control**: Closed-loop control for precise mechanism positioning
- **Soft Limits**: Hardware and software protection against over-travel
- **Manual Override**: Joystick control with smooth transition to automatic mode
- **Current Limiting**: Motor protection and optimized battery usage
- **Preset Positions**: Programmable positions for common mechanism states

### Ball Intake System (DISABLED - Example Code)
⚠️ **Note**: The ball intake subsystem is disabled as the hardware does not physically exist on this robot. The code is kept as an example for future development.

- **Variable Speed Control**: Configurable speeds for intake, eject, and hold modes
- **Current Limiting**: Motor protection
- **Multiple Operating Modes**: Intake, eject, hold, and stop commands

### Custom Utilities
- **Drivetools (DT)**: Navigation system for field positioning and path visualization
- **Math Library (M)**: Vector mathematics (2D/3D/4D) and polygon operations
- **Auto Utilities**: Helper functions for autonomous routines

##  Software Architecture

### Organized Package Structure**: Commands organized by function (auto, drivetrain, vision, actuator)
- **Framework
- **Command-Based Programming**: WPILib 2026 command-based architecture
- **Subsystem Organization**: Modular design for maintainability
- **Default Commands**: Continuous teleop control with button overrides
- **Vendor Libraries**: REVLib, PhotonLib, PathPlannerLib

### Control Systems
- **Swerve Kinematics**: Precise field-oriented drive control
- **SwerveDrivePoseEstimator**: Kalman filter for sensor fusion
- **On-Board PID**: SPARK MAX controllers for low-latency feedback loops
- **Smart Motion**: Velocity and acceleration limiting for smooth operation

### Telemetry
- **SmartDashboard Integration**: Real-time subsystem monitoring
- **Field Visualization**: Live robot pose and path display
- **Vision Diagnostics**: AprilTag detection status and pose confidence
- **Motor Telemetry**: Current draw, output, and position feedback

##  Hardware Configuration

### Electronics
| Component | Type | CAN ID / Port | Notes |
|-----------|------|---------------|-------|
| Front Left Drive | NEO (SPARK MAX) | 12 | MAXSwerve Module |
| Front Left Turn | NEO (SPARK MAX) | 13 | MAXSwerve Module |
| Front Right Drive | NEO (SPARK MAX) | 14 | MAXSwerve Module |
| Front Right Turn | NEO (SPARK MAX) | 15 | MAXSwerve Module |
| Rear Left Drive | NEO (SPARK MAX) | 10 | MAXSwerve Module |
| Rear Left Turn | NEO (SPARK MAX) | 11 | MAXSwerve Module |
| Rear Right Drive | NEO (SPARK MAX) | 16 | MAXSwerve Module |
| Rear Right Turn | NEO (SPARK MAX) | 17 | MAXSwerve Module |
| ~~Actuator~~ | ~~NEO (SPARK MAX)~~ | ~~20~~ | ⚠️ DISABLED - Example code only |
| ~~Actuator2~~ | ~~NEO (SPARK MAX)~~ | ~~21~~ | ⚠️ DISABLED - Example code only |
| ~~Ball Intake~~ | ~~NEO (SPARK MAX)~~ | ~~22~~ | ⚠️ DISABLED - Example code only |
| Gyro | NavX (SPI) | SPI | Robot heading |
| Camera 0 | PhotonVision | Network | Front AprilTag detection |
| Camera 1 | PhotonVision | Network | Back AprilTag detection |
| Camera 2 | PhotonVision | Network | Left AprilTag detection |
| Camera 3 | PhotonVision | Network | Right AprilTag detection |

### Dimensions
- **Wheelbase**: 24.5 inches (front-to-back)
- **Track Width**: 24.5 inches (left-to-right)
- **Wheel Diameter**: 3 inches (76mm)
- **Drive Ratio**: MAXSwerve 12T pinion

##  Installation

### Prerequisites
- **WPILib 2026**: [Download here](https://github.com/wpilibsuite/allwpilib/releases)
- **VS Code**: Included with WPILib installer
- **Git**: For version control
- **Java 17**: Included with WPILib

### Setup Instructions

1. **Clone the repository**
   ```bash
   git clone https://github.com/BCHRobotics/<REPO>.git
   ```

2. **Install vendor dependencies**
   - Press `Ctrl+Shift+P` in VS Code
   - Select "WPILib: Manage Vendor Libraries"
   - Select "Install new libraries (online)"
   - Check: **PhotonLib**, **REVLib**, **PathplannerLib**, **Studica (NavX)**
   - Click OK

3. **Configure robot-specific settings**
   - Open `Constants.java`
   - Set number of cameras in `VisionConstants.kNumCameras` (1-4)
   - Enable/disable cameras in `VisionConstants.kCamerasEnabled[]`
   - Set camera names in `VisionConstants.kCameraNames[]`
   - Calibrate camera transforms in `VisionConstants.kRobotToCams[]`
   - ~~Set actuator CAN ID in `ActuatorConstants.kMotorCanId`~~ (DISABLED)
   - Verify all CAN IDs match your robot wiring

4. **Build the project**
   ```bash
   ./gradlew build
   ```

5. **Deploy to robot**
   - Connect to robot via USB or WiFi
   - Press `Ctrl+Shift+P` → "WPILib: Deploy Robot Code"

##  Controls

### Driver Controller (PS5 DualSense Controller - Port 0)

#### Active Drive Controls
- **Left Stick**: Translate (strafe) the robot
- **Right Stick X**: Rotate the robot
- Field-oriented control enabled by default with slew rate limiting

#### Active Vision Controls
- **Square Button**: Navigate to AprilTag 12 at 1.0m distance (autonomous positioning, 10s timeout)
- **Options Button**: Align to AprilTag 4 for scoring (hold for continuous alignment, 5s timeout)

#### Disabled Controls (Example Code)
⚠️ The following controls are commented out as the hardware does not exist:

~~**Actuator 1 Controls (SPARK MAX PID)**~~:
- ~~Right Stick Y: Manual actuator control (scaled for precision)~~
- ~~Cross Button: Move to home position (retracted)~~
- ~~Circle Button: Move to mid position~~
- ~~Triangle Button: Move to max extension~~

~~**Actuator 2 Controls (WPILib PID)**~~:
- ~~L2 Trigger: Manual retract~~
- ~~R2 Trigger: Manual extend~~
- ~~D-Pad Down: Move to home position~~
- ~~D-Pad Left/Right: Move to mid position~~
- ~~D-Pad Up: Move to max extension~~
- ~~Create Button: Reset encoder~~

~~**Ball Intake Controls**~~:
- ~~L1 Button: Intake balls (while held)~~
- ~~R1 Button: Eject balls (while held)~~
- ~~Touchpad: Hold balls (while held)~~
- ~~PS Button: Stop intake (momentary)~~

## 📊 Vision Setup

Follow the detailed PhotonVision setup guide: [`PHOTONVISION_SETUP.md`](PHOTONVISION_SETUP.md)

### Multi-Camera Configuration
The robot supports up to 4 cameras simultaneously:
- **Camera 0**: Typically front-facing (default enabled)
- **Camera 1**: Typically back-facing (default enabled)
- **Camera 2**: Typically left-facing (default enabled)
- **Camera 3**: Typically right-facing (default enabled)

### Quick Setup (Per Camera)
1. Install PhotonVision on coprocessor (Raspberry Pi/Orange Pi)
2. Configure camera name in PhotonVision UI to match `kCameraNames[]`
3. Calibrate each camera using PhotonVision calibration tool
4. Measure and set robot-to-camera transform for each camera:
   - Translation: X (forward), Y (left), Z (up) from robot center
   - Rotation: Roll, Pitch, Yaw for camera orientation
5. Enable/disable cameras in `kCamerasEnabled[]` as needed
6. Tune standard deviations through testing
7. Verify AprilTag detection on SmartDashboard (`Vision/Cam0/`, `Vision/Cam1/`, etc.)

### Telemetry
Each camera provides individual telemetry:
- `Vision/Cam[0-3]/Has Targets` - Whether camera sees AprilTags
- `Vision/Cam[0-3]/Target Count` - Number of tags visible
- `Vision/Cam[0-3]/Best Target ID` - Closest/best tag ID
- `Vision/Cam[0-3]/Ambiguity` - Pose confidence (lower = better)
- `Vision/Total Cameras` - Number of active cameras
- `Vision/Cameras With Targets` - How many cameras see tags

### Console Diagnostics (Once Per Second)
The robot outputs comprehensive diagnostics to the console at 1Hz:
- **Robot Position**: X, Y coordinates and heading in degrees
- **Visible AprilTags**: Tag IDs, ambiguity values, distance, camera source
- **Pose Update Status**: Whether each tag is being used to update robot position
- **Motor Currents**: Drive and turn currents for all four swerve modules

## 🔬 Tuning Guide

### PID Tuning (Actuator)
1. Start with `kP = 0.1`, `kI = 0`, `kD = 0`
2. Increase kP until responsive (minimal lag)
3. If oscillating, reduce kP and add small kD (0.01-0.1)
4. Add tiny kI (0.0001) only if steady-state error exists
5. Verify smooth motion and position hold

### Vision Tuning
1. Test single-tag detection at various distances
2. Adjust `kSingleTagStdDevs` based on observed error
### Autonomous Modes
The robot includes a dashboard-selectable autonomous chooser with the following modes:

1. **Do Nothing** (Default) - Safe option for testing, robot remains stationary
2. **Drive Forward 2m** - Simple mobility autonomous, drives 2 meters forward
3. **Align to AprilTag 1** - Vision-based autonomous that aligns to AprilTag ID 1 (corrected from description)

Select autonomous mode from the "Autonomous Mode" dropdown on the Driver Station SmartDashboard.

### PathPlanner Integration
- Paths are stored in `src/main/deploy/pathplanner/paths/`
- Autos are stored in `src/main/deploy/pathplanner/autos/`
- Available PathPlanner autos (can be enabled in RobotContainer):
  - **[C] 0-2 Coral**: Center starting positions
  - **[L] 0-2 Coral**: Left starting positions
  - **[R] 0-2 Coral**: Right starting positions

### Creating New Autos
1. **Using Commands**: Create command sequences in `RobotContainer.configureAutoChooser()`
2. **Using PathPlanner**:
   - Open PathPlanner applic    # Command implementations
│   │   │   ├── auto/           # Autonomous commands
│   │   │   │   ├── DriveForwardCommand.java
│   │   │   │   └── package-info.java
│   │   │   ├── drivetrain/     # Drivetrain control commands
│   │   │   │   ├── TeleopDriveCommand.java
│   │   │   │   ├── GoToPositionCommand.java
│   │   │   │   ├── GoToPositionRelativeCommand.java
│   │   │   │   └── package-info.java
│   │   │   ├── vision/         # Vision alignment commands
│   │   │   │   ├── AlignToAprilTagCommand.java
│   │   │   │   ├── GoToAprilTagCommand.java
│   │   │   │   └── package-info.java
│   │   │   └── actuator/       # Actuator commands
│   │   │       ├── MoveToPositionCommand.java
│   │   │       └── package-info.java
│   │   ├── subsystems/         # Robot subsystems
│   │   │   ├── Actuator.java       # SPARK MAX onboard PID (DISABLED)
│   │   │   ├── Actuator2.java      # WPILib RoboRIO PID (DISABLED)
│   │   │   ├── BallIntake.java     # Ball intake subsystem (DISABLED)
│   │   │   ├── Drivetrain.java
│   │   │   ├── MAXSwerveModule.java
│   │   │   └── Vision.java
│   │   ├── Constants.java      # All robot constants
│   │   ├── Main.java
│   │   ├── Robot.java
│   │   └── RobotContainer.java # Subsystems, commands, and bindings
│   ├── utils/                  # Utility libraries
│   │   ├── DT/                 # Drivetools navigation
│   │   └── M/                  # Math utilities (Vector2/3/4, Polygon)
│   └── deploy/                 # On-robot resources
│       └── pathplanner/        # Autonomous paths
├── vendordeps/                 # Vendor library dependencies
├── build.gradle                # Build configuration
├── FUNCTIONAL_SPECIFICATION_DOCUMENT.md  # Technical specification for developers
├── PHOTONVISION_SETUP.md      # PhotonVision setup guide
├── WPILib-License.md          # WPILib license
└── README.md                  # This file
```

## 📚 Documentation

### Additional Resources
- **[FUNCTIONAL_SPECIFICATION_DOCUMENT.md](FUNCTIONAL_SPECIFICATION_DOCUMENT.md)**: Comprehensive technical specification for AI agents and developers
- **[PHOTONVISION_SETUP.md](PHOTONVISION_SETUP.md)**: Detailed PhotonVision setup and calibration guide
- **WPILib Docs**: https://docs.wpilib.org/
- **PhotonVision Docs**: https://docs.photonvision.org/
- **PathPlanner Docs**: https://pathplanner.dev/

##  Contributing

### Team Members
Team 2386 consists of students, mentors, and volunteers dedicated to FIRST robotics.

### Development Workflow
1. Create feature branch: `git checkout -b feature/my-feature`
2. Make changes and test thoroughly
3. Commit with clear messages: `git commit -m "Add feature X"`
4. Push and create pull request
5. Code review by team members
6. Merge to main after approval

### Coding Standards
- Follow WPILib style guidelines (see Code_Review.md for details)
- Use `m_` prefix for member variables, `k` prefix for constants
- Document all public methods with JavaDoc
- Use meaningful variable names
- Include units in constant names (e.g., `kSpeedMetersPerSecond`)
- Test changes on practice robot before competition bot
- Keep code readable for high school students
│   └── deploy/             # On-robot resources
│       └── pathplanner/    # Autonomous paths
├── vendordeps/             # Vendor library dependencies
├── build.gradle            # Build configuration
└── README.md              # This file
```
Number**: FRC Team 2386 / BCH Robotics
- **GitHub**: [BCHRobotics](https://github.com/BCHRobotics/)
- **Branch**: MentorExample (development), main (competition-ready)
Team 2386 consists of students, mentors, and volunteers dedicated to FIRST robotics.

### Development Workflow
1. Create feature branch: `git checkout -b feature/my-feature`
2. Make changes and test thoroughly
3. Commit with clear messages: `git commit -m "Add feature X"`
4. Push and create pull request
5. Code review by team members
6. Merge to main after approval

### Coding Standards
- Follow WPILib style guidelines (see FUNCTIONAL_SPECIFICATION_DOCUMENT.md for details)
- Use `m_` prefix for member variables, `k` prefix for constants
- Document all public methods with JavaDoc
- Use meaningful variable names
- Include units in constant names (e.g., `kSpeedMetersPerSecond`)
- Test changes on practice robot before competition bot
- Keep code readable for high school students
- Mark disabled/example code with clear comments

##  License

This project is licensed under the WPILib BSD License - see [WPILib-License.md](WPILib-License.md) for details.

## Acknowledgments

- **FIRST Robotics Competition**: For creating an amazing program
- **WPILib**: For the comprehensive robotics framework
- **REV Robotics**: For MAXSwerve modules and SPARK MAX controllers
- **PhotonVision**: For vision processing software
- **Team 2386 Mentors**: For guidance and expertise
- **Team 2386 Students**: For dedication and hard work

## 📞 Contact

- **Team Website**: [Insert team website]
- **GitHub**: [FRCTeam2386](https://github.com/XXXXX)
- **Email**: [Insert team email]
- **Social Media**: [Insert social media handles]

---

**Go Team 2386! 🚀**
