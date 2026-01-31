// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  
  public static final class OIConstants {
    public static final int kMainControllerPort = 0;
    public static final int kBackupControllerPort = 1;

    public static final double kDriveDeadband = 0.05;
    public static final double kTurnDeadband = 0.12;
    public static final double kTwistDeadband = 0.5;

    public static final boolean kFieldRelative = true;
    public static final boolean kRateLimited = true;
  }

  // swerve drive constants (dimensions, max speed, etc.)
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum and minimum capable speeds of
    // the robot, rather the allowed maximum and minimum speeds.
    public static final double maxSpeedNormal = 3.3; // 3.3
    public static final double maxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 3; // radians per second
    public static final double kMagnitudeSlewRate = 3.5; // percent per second (1 = 100%) // 3.6
    public static final double kRotationalSlewRate = 2; // percent per second (1 = 100%) // 3.0

    // aggressive rate limits
    // public static final double kDirectionSlewRate = 6;
    // public static final double kMagnitudeSlewRate = 7;
    // public static final double kRotationalSlewRate = 3.5;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 12;
    public static final int kRearLeftDrivingCanId = 10;
    public static final int kFrontRightDrivingCanId = 14;
    public static final int kRearRightDrivingCanId = 16; 

    public static final int kFrontLeftTurningCanId = 13;
    public static final int kRearLeftTurningCanId = 11;
    public static final int kFrontRightTurningCanId = 15;
    public static final int kRearRightTurningCanId = 17;

    public static final boolean kGyroReversed = true;
  }

  /**
   * Vision and PhotonVision Constants for 2026 Rebuilt
   * 
   * Contains all configuration for AprilTag detection, pose estimation,
   * and vision-based autonomous alignment.
   * 
   * Multi-Camera Support:
   * - Up to 4 cameras can be configured
   * - Each camera can be individually enabled/disabled
   * - Each camera has its own name and transform
   * - Pose estimates from all enabled cameras are fused
   */
  public static final class VisionConstants{
    // ========== Multi-Camera Configuration ==========
    
    /**
     * Number of cameras to use (1-4).
     * Only the first kNumCameras will be initialized.
     */
    public static final int kNumCameras = 4;
    
    /**
     * Enable/disable individual cameras.
     * Set to false to disable a camera without removing its configuration.
     * PLACEHOLDER: Enable cameras you have connected.
     */
    public static final boolean[] kCamerasEnabled = {
      true,  // Camera 0
      true,  // Camera 1 - banana_1 (ball detection)
      false,  // Camera 2
      false,   // Camera 3
    };
    
    /**
     * Names of cameras in PhotonVision UI.
     * PLACEHOLDER: Change these to match your camera names in PhotonVision settings.
     * Common naming: "Front_Cam", "Back_Cam", "Left_Cam", "Right_Cam"
     *               "OV9281_Front", "LifeCam_Left", etc.
     */
    public static final String[] kCameraNames = {
      "Camera_0",  // Front camera (AprilTag detection)
      "banana_1",  // Ball detection camera
      "Camera_2",  // Left camera
      "Camera_3"   // Right camera
    };
    
    /**
     * Transforms from robot center to each camera.
     * PLACEHOLDER: Measure and calibrate for your specific robot.
     * 
     * Translation(X, Y, Z):
     *   X: Forward/backward from robot center (meters, + = forward)
     *   Y: Left/right from robot center (meters, + = left)
     *   Z: Up/down from robot center (meters, + = up)
     * 
     * Rotation(Roll, Pitch, Yaw):
     *   Roll: Rotation around X axis (radians)
     *   Pitch: Rotation around Y axis (radians, - = tilted up)
     *   Yaw: Rotation around Z axis (radians)
     * 
     * Example configurations:
     * Front camera: 12" forward, 6" up, tilted 15° up:
     *   new Transform3d(
     *     new Translation3d(Units.inchesToMeters(12), 0, Units.inchesToMeters(6)),
     *     new Rotation3d(0, -Math.toRadians(15), 0)
     *   )
     * 
     * Left camera: 6" left, 8" up, facing left (90° yaw):
     *   new Transform3d(
     *     new Translation3d(0, Units.inchesToMeters(6), Units.inchesToMeters(8)),
     *     new Rotation3d(0, 0, Math.toRadians(90))
     *   )
     */
    public static final Transform3d[] kRobotToCams = {
      // Camera 0 - Front
      new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(34)),
        new Rotation3d(0, 0, 0)
      ),
      // Camera 1 - banana_1 (Ball detection)
      // PLACEHOLDER: Measure and set actual position/orientation
      new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(34)),
        new Rotation3d(0, 0, 0)
      ),
      // Camera 2 - Left
      new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation3d(0, 0, Math.toRadians(90))
      ),
      // Camera 3 - Right
      new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation3d(0, 0, Math.toRadians(-90))
      )
    };
    
    // ========== Pose Estimation Tuning ==========
    
    /**
     * Maximum pose ambiguity to accept vision measurements.
     * 
     * Ambiguity measures how "confident" PhotonVision is in the pose estimate.
     * Lower values = more confident. Range: 0.0 (perfect) to 1.0 (terrible).
     * 
     * Recommended: 0.2 for competition, 0.3 for testing
     * If you get too many rejected poses, increase this value.
     */
    public static final double kMaxAmbiguity = 0.3;
    
    /**
     * Standard deviations for single-tag pose estimates.
     * 
     * Matrix format: [x_stddev, y_stddev, theta_stddev]
     * Units: meters for x/y, radians for theta
     * 
     * Higher values = trust vision less (rely more on odometry)
     * Lower values = trust vision more (rely less on odometry)
     * 
     * PLACEHOLDER: Tune these values through testing:
     * 1. Drive robot to known position
     * 2. Observe vision estimate error
     * 3. Adjust standard deviations accordingly
     * 
     * Recommended starting values: 0.7m for XY, 0.9 rad for theta
     */
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.7, 0.7, 0.9);
    
    /**
     * Standard deviations for multi-tag pose estimates.
     * 
     * When multiple AprilTags are visible, the estimate is much more reliable.
     * These values should be significantly lower than single-tag values.
     * 
     * PLACEHOLDER: Tune through testing (typically 1/3 to 1/5 of single-tag values)
     * Recommended starting values: 0.2m for XY, 0.3 rad for theta
     */
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.3);
    
    /**
     * Distance weight factor for increasing uncertainty.
     * 
     * As distance to AprilTag increases, vision becomes less accurate.
     * This factor scales the standard deviations based on distance.
     * 
     * Formula: stddev *= (1 + distance^2 * kDistanceWeight)
     * 
     * PLACEHOLDER: Tune based on your camera's resolution and optics
     * Higher values = more aggressive uncertainty scaling
     * Recommended starting value: 0.01
     */
    public static final double kDistanceWeight = 0.01;
    
    // ========== Vision Alignment Constants ==========
    
    /**
     * Allowed position error during vision alignment (meters).
     * 
     * These define when the robot is "close enough" to the target position.
     * Cannot be zero or commands will never complete (robot can't be perfect).
     */
    public static final double allowedXError = 0.025; // 5cm tolerance
    public static final double allowedYError = 0.025; // 5cm tolerance
    
    /**
     * PID constants for X-axis alignment to AprilTags.
     * 
     * Used when driving toward or away from tags.
     * PLACEHOLDER: Tune through testing (start low, increase until responsive)
     */
    public static double kAlignP = 0.6;
    public static double kAlignI = 0;
    public static double kAlignD = 0;
    
    /**
     * PID constants for rotational alignment (heading lock and vision alignment).
     * 
     * Used for turning to face AprilTags.
     * PLACEHOLDER: Tune through testing
     */
    public static double kRotP = 0.018;
    public static double kRotI = 0.00001;
    public static double kRotD = 0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0746;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.07;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0.01;
    public static final double kDrivingFF = 0.25; //1 / kDriveWheelFreeSpeedRps; // why is this comment here it isn't true?????
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.65;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AutoConstants {
    // the PID constants that PathPlanner uses to drive the robot
    public static final PIDConstants translationConstants = new PIDConstants(2, 1, 0);
    public static final PIDConstants rotationConstants = new PIDConstants(1, 0, 0);
  }

  /**
   * Constants for autonomous navigation commands (GoToPosition, GoToPositionRelative, etc.).
   * 
   * These PID values control the robot's behavior when navigating to specific field positions.
   * Higher P values = more aggressive, faster response but may overshoot
   * Lower P values = smoother, more stable but slower response
   * 
   * Tune these values through testing:
   * 1. Start with low P values and gradually increase
   * 2. Add D if oscillation occurs
   * 3. Only add I if there's persistent steady-state error
   */
  public static final class NavigationConstants {
    // ========== PID Gains ==========
    
    /**
     * PID gains for X position control (field-relative).
     * Controls forward/backward movement accuracy.
     */
    public static final double kPositionP = 0.4;
    public static final double kPositionI = 0.05;
    public static final double kPositionD = 0.0;
    
    /**
     * PID gains for rotation control.
     * Controls turning accuracy to target heading.
     * Note: Lower P value than position because rotation is more sensitive.
     */
    public static final double kRotationP = 0.025;
    public static final double kRotationI = 0.005;
    public static final double kRotationD = 0.0;
    
    // ========== Tolerances ==========
    
    /**
     * Position tolerance in meters.
     * Command finishes when robot is within this distance of target.
     */
    public static final double kPositionTolerance = 0.1; // meters
    
    /**
     * Rotation tolerance in degrees.
     * Command finishes when robot heading is within this angle of target.
     */
    public static final double kRotationTolerance = 5.0; // degrees
    
    // ========== Speed Limits ==========
    
    /**
     * Maximum linear speed during autonomous navigation.
     * Limits how fast the robot can translate to target position.
     */
    public static final double kMaxLinearSpeed = 5.0; // m/s
    
    /**
     * Maximum angular speed during autonomous navigation.
     * Limits how fast the robot can rotate to target heading.
     */
    public static final double kMaxAngularSpeed = 1.0; // rad/s
    
    // ========== Field Dimensions ==========
    
    /**
     * FRC 2026 Rebuilt field length in meters.
     * Used for alliance-relative coordinate mirroring.
     */
    public static final double kFieldLength = 16.54; // meters (54.27 feet)
    
    /**
     * FRC 2026 Rebuilt field width in meters.
     * Used for alliance-relative coordinate mirroring.
     */
    public static final double kFieldWidth = 8.21; // meters (26.94 feet)
  }

  /**
   * Constants for Ball Tracking and Alignment.
   * 
   * PID control for rotating the robot to align with detected balls
   * using vision-based yaw angle feedback.
   */
  public static final class BallTrackingConstants {
    // ========== PID Gains ==========
    
    /**
     * Proportional gain for ball alignment rotation control.
     * Higher values = faster response to yaw error.
     * Tune this first - start low and increase until responsive.
     */
    public static final double kRotationP = 0.5;
    
    /**
     * Integral gain for ball alignment rotation control.
     * Helps eliminate steady-state error.
     * Usually keep low to avoid oscillation.
     */
    public static final double kRotationI = 0.0;
    
    /**
     * Derivative gain for ball alignment rotation control.
     * Reduces overshoot and oscillation.
     * Add if system oscillates with just P.
     */
    public static final double kRotationD = 0.002;
    
    // ========== Tolerances ==========
    
    /**
     * Yaw tolerance in degrees.
     * Robot is considered aligned when ball yaw is within this tolerance.
     */
    public static final double kYawTolerance = 2.0; // degrees
    
    // ========== Speed Limits ==========
    
    /**
     * Maximum rotation speed during ball tracking.
     * Limits how fast the robot can turn while aligning to ball.
     */
    public static final double kMaxRotationSpeed = 0.4; // rad/s
  }

  /**
   * Constants for PID-controlled Actuator subsystem.
   * 
   * Configure these values for your specific mechanism:
   * - Motor CAN ID and direction
   * - PID gains (tune through testing)
   * - Physical limits (soft limits, max velocity/acceleration)
   * - Current limit (based on mechanism load)
   * - Conversion factors (if using custom units instead of rotations)
   */
  public static final class ActuatorConstants {
    // ========== Hardware Configuration ==========
    
    /**
     * CAN ID of the SPARK MAX motor controller.
     * PLACEHOLDER: Set to match your wiring.
     */
    public static final int kMotorCanId = 20;
    
    /**
     * Motor inversion.
     * PLACEHOLDER: Test and set based on desired positive direction.
     * true = inverted, false = not inverted
     */
    public static final boolean kMotorInverted = false;
    
    /**
     * Current limit in amps.
     * 
     * Protects motor and mechanism from overcurrent.
     * PLACEHOLDER: Tune based on mechanism load.
     * - Light mechanisms (small arms): 20-30A
     * - Medium mechanisms (elevators): 30-40A
     * - Heavy mechanisms (climbers): 40-60A
     */
    public static final int kCurrentLimit = 30;
    
    // ========== PID Configuration ==========
    
    /**
     * Proportional gain for position control.
     * 
     * How aggressively the controller responds to position error.
     * PLACEHOLDER: Start low (0.1) and increase until responsive without oscillation.
     */
    public static final double kP = 0.5;
    
    /**
     * Integral gain for position control.
     * 
     * Eliminates steady-state error (useful for holding against gravity/load).
     * PLACEHOLDER: Usually 0 or very small (0.0001-0.001). Add only if needed.
     */
    public static final double kI = 0.0;
    
    /**
     * Derivative gain for position control.
     * 
     * Dampens oscillations and overshoot.
     * PLACEHOLDER: Start at 0. Add small values (0.01-0.1) if oscillating.
     */
    public static final double kD = 0.0;
    
    /**
     * Minimum output (reverse direction).
     * 
     * Limits maximum reverse power. Usually -1.0 for full range.
     */
    public static final double kMinOutput = -1.0;
    
    /**
     * Maximum output (forward direction).
     * 
     * Limits maximum forward power. Usually 1.0 for full range.
     */
    public static final double kMaxOutput = 1.0;
    
    // ========== Motion Constraints ==========
    
    /**
     * Maximum velocity in rotations per second.
     * 
     * Limits how fast the mechanism moves (for smooth motion).
     * PLACEHOLDER: NEO free speed is ~5676 RPM ≈ 95 RPS. Start at 20-50 RPS.
     */
    public static final double kMaxVelocity = 30; // rotations per second
    
    /**
     * Maximum acceleration in rotations per second².
     * 
     * Limits how quickly velocity changes (prevents jerky motion).
     * PLACEHOLDER: Start at kMaxVelocity * 2, adjust for smoothness.
     */
    public static final double kMaxAcceleration = 60; // rotations per second²
    
    // ========== Position Limits ==========
    
    /**
     * Forward soft limit in rotations.
     * 
     * Maximum extension position. Motor will not move beyond this.
     * PLACEHOLDER: Measure full extension of your mechanism in motor rotations.
     * Set slightly before physical hard stop for safety.
     */
    public static final double kForwardSoftLimit = 50; // rotations
    
    /**
     * Reverse soft limit in rotations.
     * 
     * Minimum retraction position (usually 0 at home).
     * PLACEHOLDER: Typically 0 if encoder is zeroed at fully retracted position.
     * Use negative value if mechanism can go below zero point.
     */
    public static final double kReverseSoftLimit = 0; // rotations
    
    /**
     * Position tolerance in rotations.
     * 
     * How close to setpoint is "close enough".
     * PLACEHOLDER: Set based on mechanism precision needs.
     * Tighter tolerance = more precise but may never finish.
     */
    public static final double kPositionTolerance = 1.0; // rotations
    
    // ========== Unit Conversion Factors ==========
    
    /**
     * Position conversion factor.
     * 
     * Converts encoder rotations to custom units (inches, degrees, etc.).
     * Default: 1.0 (rotations)
     * 
     * Examples:
     * - Linear mechanism with 2-inch pitch pulley: 2 * Math.PI (inches per rotation)
     * - Geared arm with 100:1 ratio: 360.0 / 100 (degrees per motor rotation)
     * 
     * PLACEHOLDER: Calculate based on your mechanism geometry.
     */
    public static final double kPositionConversionFactor = 1.0;
    
    /**
     * Velocity conversion factor.
     * 
     * Converts encoder RPM to custom units per second.
     * Default: 1.0 / 60.0 (rotations per second)
     * 
     * If using custom position units, set to: kPositionConversionFactor / 60.0
     * 
     * PLACEHOLDER: Calculate to match position units.
     */
    public static final double kVelocityConversionFactor = 1.0 / 60.0;
    
    // ========== Preset Positions ==========
    
    /**
     * Home position (fully retracted).
     */
    public static final double kHomePosition = 0.0;
    
    /**
     * Mid-range position.
     * PLACEHOLDER: Set based on mechanism requirements.
     */
    public static final double kMidPosition = 25.0;
    
    /**
     * Maximum extension position.
     * PLACEHOLDER: Should be at or below kForwardSoftLimit.
     */
    public static final double kMaxPosition = 50.0;
    
    // ========== Controller Configuration ==========
    
    /**
     * Deadband for manual joystick control.
     * 
     * Joystick values below this are treated as zero.
     * Prevents drift from imperfect joystick centering.
     */
    public static final double kManualDeadband = 0.1;
    
    /**
     * Scaling factor for manual control speed.
     * 
     * Multiplied by joystick input to limit maximum manual speed.
     * Lower values = slower, more precise control.
     */
    public static final double kManualSpeedScale = 0.5;
  }

  /**
   * Constants for WPILib PID-controlled Actuator2 subsystem.
   * 
   * This actuator uses WPILib's PIDController instead of SPARK MAX onboard PID.
   * Control loop runs on the RoboRIO, providing more flexibility and easier debugging.
   * 
   * Configure these values for your specific mechanism:
   * - Motor CAN ID and direction
   * - PID gains (tune through testing, adjustable via SmartDashboard)
   * - Physical limits (soft limits enforced in software)
   * - Current limit (based on mechanism load)
   * - Conversion factors (if using custom units instead of rotations)
   * 
   * Advantages of RoboRIO PID:
   * - Real-time PID tuning via SmartDashboard
   * - Better telemetry and logging
   * - Easier custom control algorithms
   * - Can integrate with other subsystem logic
   */
  public static final class Actuator2Constants {
    // ========== Hardware Configuration ==========
    
    /**
     * CAN ID of the SPARK MAX motor controller.
     * PLACEHOLDER: Set to match your wiring (must be different from Actuator 1).
     */
    public static final int kMotorCanId = 21;
    
    /**
     * Motor inversion.
     * PLACEHOLDER: Test and set based on desired positive direction.
     * true = inverted, false = not inverted
     */
    public static final boolean kMotorInverted = false;
    
    /**
     * Current limit in amps.
     * 
     * Protects motor and mechanism from overcurrent.
     * PLACEHOLDER: Tune based on mechanism load.
     * - Light mechanisms (small arms): 20-30A
     * - Medium mechanisms (elevators): 30-40A
     * - Heavy mechanisms (climbers): 40-60A
     */
    public static final int kCurrentLimit = 30;
    
    // ========== WPILib PID Configuration ==========
    
    /**
     * Proportional gain for position control.
     * 
     * How aggressively the controller responds to position error.
     * PLACEHOLDER: Start low (0.05-0.1) and increase until responsive without oscillation.
     * 
     * NOTE: RoboRIO PID typically needs lower gains than onboard PID due to:
     * - Slower update rate (~20ms vs ~1ms)
     * - Communication latency to motor controller
     * 
     * Tune via SmartDashboard for real-time adjustment.
     */
    public static final double kP = 0.1;
    
    /**
     * Integral gain for position control.
     * 
     * Eliminates steady-state error (useful for holding against gravity/load).
     * PLACEHOLDER: Usually 0 or very small (0.00001-0.0001). Add only if needed.
     * 
     * WARNING: Integral windup can cause overshoot. Use cautiously.
     */
    public static final double kI = 0.0;
    
    /**
     * Derivative gain for position control.
     * 
     * Dampens oscillations and overshoot.
     * PLACEHOLDER: Start at 0. Add small values (0.001-0.01) if oscillating.
     * 
     * Can help stabilize systems with low P gain.
     */
    public static final double kD = 0.0;
    
    /**
     * Minimum output (reverse direction).
     * 
     * Limits maximum reverse power. Usually -1.0 for full range.
     * Can reduce if mechanism doesn't need full power.
     */
    public static final double kMinOutput = -1.0;
    
    /**
     * Maximum output (forward direction).
     * 
     * Limits maximum forward power. Usually 1.0 for full range.
     * Can reduce if mechanism doesn't need full power.
     */
    public static final double kMaxOutput = 1.0;
    
    // ========== Position Limits ==========
    
    /**
     * Forward soft limit in rotations.
     * 
     * Maximum extension position. Enforced in software.
     * PLACEHOLDER: Measure full extension of your mechanism in motor rotations.
     * Set slightly before physical hard stop for safety.
     */
    public static final double kForwardSoftLimit = 50; // rotations
    
    /**
     * Reverse soft limit in rotations.
     * 
     * Minimum retraction position (usually 0 at home).
     * PLACEHOLDER: Typically 0 if encoder is zeroed at fully retracted position.
     * Use negative value if mechanism can go below zero point.
     */
    public static final double kReverseSoftLimit = 0; // rotations
    
    /**
     * Position tolerance in rotations.
     * 
     * How close to setpoint is "close enough" for atSetpoint() to return true.
     * PLACEHOLDER: Set based on mechanism precision needs.
     * Tighter tolerance = more precise but may never finish.
     * Looser tolerance = easier to reach but less accurate.
     */
    public static final double kPositionTolerance = 1.0; // rotations
    
    // ========== Unit Conversion Factors ==========
    
    /**
     * Position conversion factor.
     * 
     * Converts encoder rotations to custom units (inches, degrees, etc.).
     * Default: 1.0 (rotations)
     * 
     * Examples:
     * - Linear mechanism with 2-inch pitch pulley: 2 * Math.PI (inches per rotation)
     * - Geared arm with 100:1 ratio: 360.0 / 100 (degrees per motor rotation)
     * 
     * PLACEHOLDER: Calculate based on your mechanism geometry.
     */
    public static final double kPositionConversionFactor = 1.0;
    
    /**
     * Velocity conversion factor.
     * 
     * Converts encoder RPM to custom units per second.
     * Default: 1.0 / 60.0 (rotations per second)
     * 
     * If using custom position units, set to: kPositionConversionFactor / 60.0
     * 
     * PLACEHOLDER: Calculate to match position units.
     */
    public static final double kVelocityConversionFactor = 1.0 / 60.0;
    
    // ========== Preset Positions ==========
    
    /**
     * Home position (fully retracted).
     */
    public static final double kHomePosition = 0.0;
    
    /**
     * Mid-range position.
     * PLACEHOLDER: Set based on mechanism requirements.
     */
    public static final double kMidPosition = 25.0;
    
    /**
     * Maximum extension position.
     * PLACEHOLDER: Should be at or below kForwardSoftLimit.
     */
    public static final double kMaxPosition = 50.0;
    
    // ========== Controller Configuration ==========
    
    /**
     * Deadband for manual joystick control.
     * 
     * Joystick values below this are treated as zero.
     * Prevents drift from imperfect joystick centering.
     */
    public static final double kManualDeadband = 0.1;
    
    /**
     * Scaling factor for manual control speed.
     * 
     * Multiplied by joystick input to limit maximum manual speed.
     * Lower values = slower, more precise control.
     */
    public static final double kManualSpeedScale = 0.5;
  }

  /**
   * DISABLED: Ball Intake Subsystem Constants
   * 
   * Motor CAN ID 22 does not physically exist on this robot.
   * These stub constants prevent compile errors in BallIntake.java.
   * The subsystem is disabled in RobotContainer.java and will not be instantiated.
   */
  public static final class BallIntakeConstants {
    // Stub constants to prevent compile errors (subsystem disabled)
    public static final int kMotorCanId = 99;  // Invalid CAN ID - DO NOT USE
    public static final boolean kMotorInverted = false;
    public static final int kCurrentLimit = 25;
    public static final double kIntakeSpeed = 0.0;  // Disabled
    public static final double kEjectSpeed = 0.0;   // Disabled
    public static final double kHoldSpeed = 0.0;    // Disabled
    public static final double kSpeedDeadband = 0.05;
  }
}

