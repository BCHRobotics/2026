# SmartDashboard Configuration

This document lists all the network table keys published to SmartDashboard/Shuffleboard by the robot code.

## Drivetrain
| Key | Type | Description | Source File |
|---|---|---|---|
| `Gyro Heading` | Number | The current gyro heading of the robot in degrees. | `src/main/java/frc/robot/subsystems/Drivetrain.java` |

## Vision
### General
| Key | Type | Description | Source File |
|---|---|---|---|
| `Vision Field` | Data (Field2d) | Field2d widget showing the robot's vision-estimated pose. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Odometry Field` | Data (Field2d) | Field2d widget showing the robot's odometry-estimated pose. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Total Cameras` | Number | Total number of configured active cameras. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Cameras With Targets` | Number | Number of cameras currently detecting targets. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Total Targets` | Number | Total number of targets detected across all cameras. | `src/main/java/frc/robot/subsystems/Vision.java` |

### Per-Camera Telemetry
For each camera `i` (e.g., `Vision/Cam0/`):
| Key | Type | Description | Source File |
|---|---|---|---|
| `Vision/Cam{i}/Has Targets` | Boolean | Whether the camera currently detects any targets. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Cam{i}/Target Count` | Number | Number of targets detected by this camera. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Cam{i}/Timestamp` | Number | Timestamp of the latest pipeline result. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Cam{i}/Name` | String | Name of the camera (e.g., "Camera_0"). | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Cam{i}/Best Target ID` | Number | Fiducial ID of the best detected target. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Cam{i}/Ambiguity` | Number | Pose ambiguity of the best detected target. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Cam{i}/Area` | Number | Area of the best detected target in screen space. | `src/main/java/frc/robot/subsystems/Vision.java` |
| `Vision/Cam{i}/Alliance` | String | Alliance color of the best target ("Blue" or "Red"). | `src/main/java/frc/robot/subsystems/Vision.java` |

## Vision Web Server
| Key | Type | Description | Source File |
|---|---|---|---|
| `WebServer/Running` | Boolean | Status of the integrated vision web server. | `src/main/java/frc/robot/webserver/VisionWebServer.java` |
| `WebServer/Port` | Number | Port number the web server is listening on (default: 8082). | `src/main/java/frc/robot/webserver/VisionWebServer.java` |

## Autonomous & Configuration
| Key | Type | Description | Source File |
|---|---|---|---|
| `Auto Mode` | SendableChooser | Dropdown to select the autonomous routine. | `src/main/java/frc/robot/RobotContainer.java` |
| `PP Translation PID` | SendableChooser | Dropdown to select PathPlanner Translation PID constants. | `src/main/java/frc/robot/RobotContainer.java` |
| `PP Rotation PID` | SendableChooser | Dropdown to select PathPlanner Rotation PID constants. | `src/main/java/frc/robot/RobotContainer.java` |

## Note on Vision Web Server JSON API
The `VisionWebServer` reads several keys to generate its JSON API response. Ideally, these keys should be populated by the `Vision` subsystem if they are not already.

**Keys read by WebServer (expected to be present):**
- `Vision/Camera_{i}/Enabled`
- `Vision/Camera_{i}/Latency`
- `Vision/AprilTag/Count`
- `Vision/AprilTag/BestID`
- `Vision/AprilTag/BestDistance`
- `Vision/AprilTag/BestYaw`
- `Vision/Ball/Visible`
- `Vision/Ball/Yaw`
- `Vision/Ball/Pitch`
- `Vision/Ball/Area`
- `Vision/HasVisionPose`
- `Vision/EstimatedPose/X`
- `Vision/EstimatedPose/Y`
- `Vision/EstimatedPose/Rotation`
- `Vision/PoseTimestamp`
