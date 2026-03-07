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
  
  // Opoerator Interface
  public static final class OIConstants {
    public static final int kMainControllerPort = 0;
    public static final int kBackupControllerPort = 1;

    public enum ControllerType {
      PS5,
      XBOX
    }
    public static final ControllerType kDriverControllerType = ControllerType.PS5;

    public static final double kDriveDeadband = 0.05;
    public static final double kTurnDeadband = 0.12;
    public static final double kTwistDeadband = 0.5;

    public static final boolean kFieldRelative = true;
    public static final boolean kRateLimited = true;
  }

  // Drive Constants
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum and minimum capable speeds of
    // the robot, rather the allowed maximum and minimum speeds.
    public static final double maxSpeedNormal = 3.3; // 3.3
    public static final double maxAngularSpeed = 2.0 * Math.PI; // radians per second

    // Slew rate limits
    public static final double kDirectionSlewRate = 3; // rad/s
    public static final double kMagnitudeSlewRate = 3.5; // percent/sec (1 = 100%) 
    public static final double kRotationalSlewRate = 2; // percent/sec (1 = 100%) 

    // Chassis dimensions
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    
    // Swerve drive kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Module angular offsets (radians)
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
   * - Each >can be individually enabled/disabled
   * - Each camera has its own name and transform
   * - Pose estimates from all enabled cameras are fused
   */
  public static final class VisionConstants{
    // Number of cameras to use
    public static final int kNumCameras = 4;
    
    // Camera enable flags
    public static final boolean[] kCamerasEnabled = {
      true,  // Left_Camera
      true,  // Front_Camera
      true,  // Rear_Left_Camera
      true,   // Rear_Right_Camera
    };
    
    // Camera names
    public static final String[] kCameraNames = {
      "Left_Camera",  // Left_Camera
      "Front_Camera",  // Front_Camera
      "Rear_Left_Camera",  // Rear_Left_Camera
      "Rear_Right_Camera"   // Rear_Right_Camera
    };
    
    // Transforms from robot center to each camera (in meters and radians)
    public static final Transform3d[] kRobotToCams = {
      // Left camera - Side
      new Transform3d(
        new Translation3d(Units.inchesToMeters(5.00), Units.inchesToMeters(11.5), Units.inchesToMeters(32)),
        new Rotation3d(0, 0, Math.PI/2)
      ),
      // Top_Camera - Front
      new Transform3d(
        new Translation3d(Units.inchesToMeters(6.5), Units.inchesToMeters(0), Units.inchesToMeters(36)),
        new Rotation3d(0, 0, Math.toRadians(0))
      ),
      // Rear_Left_Camera 
      new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(4.25), Units.inchesToMeters(36)),
                      new Rotation3d(0, 0, Math.toRadians(180))),

      // Rear_Right_Camera
      new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-4), Units.inchesToMeters(36)),
                      new Rotation3d(0, 0, Math.toRadians(180)))
    };
    
    // Pose estimation tuning
    public static final double kMaxAmbiguity = 0.2;
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.35, 0.35, 0.45);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.15);
    public static final double kDistanceWeight = 0.01;
    
  // Maximum distance (meters) at which we will accept a vision pose update.
  //  Measurements from targets farther than this are ignored to avoid
  //  using excessively noisy detections for pose fusion.
  public static final double kMaxTargetDistance = 3.0;
    
    // Allowed error tolerances
    public static final double allowedXError = 0.025; // 5cm tolerance
    public static final double allowedYError = 0.025; // 5cm tolerance
    
    // PID constants for X-axis alignment to AprilTags.
    public static double kAlignP = 0.6;
    public static double kAlignI = 0;
    public static double kAlignD = 0;
    
    // PID constants for rotational alignment (heading lock and vision alignment).
    public static double kRotP = 0.018;
    public static double kRotI = 0.00001;
    public static double kRotD = 0;
  }

  // Swerve Module Constants
  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 12;
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0746;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

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
    public static final double kDrivingFF = 0.25;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.65;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final int kDrivingMotorCurrentLimit = 60; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  // Neo Motor Constants
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // Vortex Motor Constants
  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  // PID constants that PathPlanner uses to drive the robot
  public static final class AutoConstants {
    // the PID constants that PathPlanner uses to drive the robot
    public static final PIDConstants translationConstants = new PIDConstants(2, 0.1, 0);
    public static final PIDConstants rotationConstants = new PIDConstants(1, 0, 0);
  }

  // Navigation Constants
  public static final class NavigationConstants {
    /**
     * PID gains for X position control (field-relative).
     * Controls forward/backward movement accuracy.
     */
    public static final double kPositionP = 1.0;
    public static final double kPositionI = 0.00;
    public static final double kPositionD = 0.01;
    
    /**
     * PID gains for rotation control.
     * Controls turning accuracy to target heading.
     */
    public static final double kRotationP = 0.025;
    public static final double kRotationI = 0.005;
    public static final double kRotationD = 0.0;
 
    public static final double kPositionTolerance = 0.05; // meters
    public static final double kRotationTolerance = 5.0; // degrees

    public static final double kMaxLinearSpeed = 5.0; // m/s
    public static final double kMaxAngularSpeed = 1.0; // rad/s

    public static final double kFieldLength = 16.54; // meters (54.27 feet)
    public static final double kFieldWidth = 8.21; // meters (26.94 feet)
  }

  /**
   * PID constants for ball alignment rotation control.
   * Used for vision-based yaw alignment of the robot to balls.
   */
  public static final class BallTrackingConstants {
    public static final double kRotationP = 0.02;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.02;

    public static final double kYawTolerance = 3.0; // degrees
    public static final double kMaxRotationSpeed = 0.3; // rad/s
  }

  public static final class BallIntakeConstants {
    // CAN IDs
    public static final int kExtendMotorCanId = 4;
    public static final int kRunMotorCanId = 6;

    // Motor inversion
    public static final boolean kExtendMotorInverted = false;
    public static final boolean kRunMotorInverted = false;

    // Current limits (amps)
    public static final int kExtendCurrentLimit = 40;
    public static final int kRunCurrentLimit = 40;

    // Extend/retract speeds (positive = extend, negative = retract)
    public static final double kExtendSpeed = 0.5;
    public static final double kRetractSpeed = -0.5;

    // Run motor speeds (positive = intake, negative = eject)
    public static final double kRunSpeed = 0.8;
    public static final double kEjectSpeed = -0.8;

    // Calibration: slow retract until current spike or timeout
    public static final double kCalibrateSpeed = -0.2;
    public static final double kCalibrateCurrentThreshold = 15.0; // amps
    public static final double kCalibrateTimeoutSeconds = 3.0;

    // How long to drive extend/retract motor before auto-stopping (seconds)
    public static final double kExtendTimeoutSeconds = 1.5;
  }

  public static final class ShooterConstants {
    public static final int FEEDER_CAN_ID = 1;
    public static final int SHOOTER1_CAN_ID = 3;
    public static final int SHOOTER2_CAN_ID = 2;
  }
}