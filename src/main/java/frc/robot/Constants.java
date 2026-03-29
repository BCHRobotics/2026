// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final int kOperatorControllerPort = 1;

    public enum ControllerType {
      PS5,
      XBOX
    }
    public static final ControllerType kDriverControllerType = ControllerType.PS5;
    public static final ControllerType kOperatorControllerType = ControllerType.PS5;

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
    public static final double maxSpeedNormal = 5.0; // 3.3
    public static final double maxSpeedTurbo = 7.0; 
    public static final double maxAngularSpeed = 2.0 * Math.PI; // radians per second

    // Slew rate limits
    public static final double kDirectionSlewRate = 3; // rad/s
    public static final double kMagnitudeSlewRate = 7.0; // percent/sec (1 = 100%) 
    public static final double kRotationalSlewRate = 2; // percent/sec (1 = 100%) 

    // Chassis dimensions
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    
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
      true,  // ShooterLeft_Camera
      true,  // ShooterRight_Camera
      // true,  // Rear_Left_Camera
      // true,   // Rear_Right_Camera
    };
    
    // Camera names
    public static final String[] kCameraNames = {
      "ShooterLeft_Camera",  //ShooterLeft_Camera; facing robot forward, intake side
      "ShooterRight_Camera",  //ShooterRight_Camera
      // "Rear_Left_Camera",  // Rear_Left_Camera
      // "Rear_Right_Camera"   // Rear_Right_Camera
    };
    
    // Transforms from robot center to each camera (in meters and radians)
    public static final Transform3d[] kRobotToCams = {
      // ShooterLeft_Camera
      new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.475), Units.inchesToMeters(11.253), Units.inchesToMeters(20.783)),
        new Rotation3d(0, 0, Math.toRadians(150))
      ),
      // ShooterRight_Camera
      new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.475), Units.inchesToMeters(-11.253), Units.inchesToMeters(20.783)),
        new Rotation3d(0, 0, Math.toRadians(-150))
      ),
      // // Rear_Left_Camera 
      // new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(4.25), Units.inchesToMeters(36)),
      //                 new Rotation3d(0, 0, Math.toRadians(180))),

      // // Rear_Right_Camera
      // new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-4), Units.inchesToMeters(36)),
      //                 new Rotation3d(0, 0, Math.toRadians(180)))
    };
    
    // Pose estimation tuning
    public static final double kMaxAmbiguity = 0.2;
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.35, 0.35, 0.45);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.15);
    public static final double kDistanceWeight = 0.01;
    public static final double kRotationDistanceWeight = 0.03;
    
    // Maximum camera-to-tag distance (meters) at which we will accept vision updates.
    public static final double kMaxSingleTagDistance = 4.0;
    public static final double kMaxMultiTagDistance = 4.0;

    // Reject measurements that jump too far from the current fused pose estimate.
    public static final double kMaxSingleTagPoseDeltaMeters = 0.75;
    public static final double kMaxMultiTagPoseDeltaMeters = 99.0;
    public static final double kMaxSingleTagRotationDeltaDegrees = 15.0;
    public static final double kMaxMultiTagRotationDeltaDegrees = 300.0;
    
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
    public static final double kWheelDiameterMeters = 0.0762;
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
    public static final double kPositionP = 0.8;
    public static final double kPositionI = 0.00;
    public static final double kPositionD = 0.02;
    
    /**
     * PID gains for rotation control.
     * Controls turning accuracy to target heading.
     */
    public static final double kRotationP = 0.031/2;
    public static final double kRotationI = 0.00;
    public static final double kRotationD = 0.00;
 
    public static final double kPositionTolerance = 0.05; // meters
    public static final double kRotationTolerance = 5.0; // degrees

    public static final double kMaxLinearSpeed = 5.0; // m/s
    public static final double kMaxAngularSpeed = 1.0; // rad/s

    public static final double kFieldLength = 16.54; // meters (54.27 feet)
    public static final double kFieldWidth = 8.21; // meters (26.94 feet)

    // 2026 Rebuilt hub centers in field coordinates (blue-origin).
    // Red hub center is provided by measurement; blue hub center is mirrored using
    // the 2026 field layout length from APRILTAG_FIELD_LAYOUT.md (16.518 m).
    public static final double kRedAllianceHubCenterX = 11.945;
    public static final double kRedAllianceHubCenterY = 4.029;
    public static final double kBlueAllianceHubCenterX = 16.518 - kRedAllianceHubCenterX;
    public static final double kBlueAllianceHubCenterY = kRedAllianceHubCenterY;

    public static final Translation2d kRedAllianceHubCenter =
      new Translation2d(kRedAllianceHubCenterX, kRedAllianceHubCenterY);
    public static final Translation2d kBlueAllianceHubCenter =
      new Translation2d(kBlueAllianceHubCenterX, kBlueAllianceHubCenterY);
  }

  public static final class VisionTuningConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.75;
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

  public static final class ClimbConstants {
    public static final int kMotorCanId = 40;
    public static final int kProximitySwitchChannel = 1;
    public static final boolean kProximitySwitchActiveLow = true;
    public static final boolean kMotorInverted = false;
    public static final int kMotorCurrentLimit = 60;
    public static final double kExtendSpeed = 1.0;
    public static final double kRetractSpeed = -0.6;
    public static final double kCurrentSpikeThresholdAmps = 40.0;
    public static final double kCurrentAverageWindowSeconds = 0.1;

    // The requested starting locations are measured from the field walls, then converted to
    // WPILib field coordinates so commands can use the same Pose2d math as the rest of the robot.
    public static final double kStartDistanceFromEndWallMeters = Units.inchesToMeters(41.5);
    public static final double kStartDistanceFromSideWallMeters = Units.inchesToMeters(120.0);
    public static final double kDriveForwardDistanceMeters = Units.inchesToMeters(20.0);
    public static final double kDriveMaxSpeedMetersPerSecond = 0.5;

    public static final Pose2d kBlueLeftStartPose = new Pose2d(
        kStartDistanceFromEndWallMeters,
        NavigationConstants.kFieldWidth - kStartDistanceFromSideWallMeters,
        Rotation2d.fromDegrees(-90.0));
    public static final Pose2d kBlueRightStartPose = new Pose2d(
        kStartDistanceFromEndWallMeters,
        kStartDistanceFromSideWallMeters,
        Rotation2d.fromDegrees(90.0));
    public static final Pose2d kRedLeftStartPose = new Pose2d(
        NavigationConstants.kFieldLength - kStartDistanceFromEndWallMeters,
        kStartDistanceFromSideWallMeters,
        Rotation2d.fromDegrees(90.0));
    public static final Pose2d kRedRightStartPose = new Pose2d(
        NavigationConstants.kFieldLength - kStartDistanceFromEndWallMeters,
        NavigationConstants.kFieldWidth - kStartDistanceFromSideWallMeters,
        Rotation2d.fromDegrees(90.0));
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
    public static final double kExtendSpeed = 0.1;
    public static final double kRetractSpeed = -0.1;

    // Extend position control
    public static final double kExtendPositionConversionFactor = 1.0;
    public static final double kExtendVelocityConversionFactor = 1.0;
    public static final double kExtendedPosition = 16.8;
    public static final double kRetractedPosition = 0.0;
    public static final double kExtendPositionTolerance = 1.0;
    public static final double kExtendPositionP = 0.08;
    public static final double kExtendPositionI = 0.0;
    public static final double kExtendPositionD = 0.0;
    public static final ClosedLoopSlot kExtendClosedLoopSlot = ClosedLoopSlot.kSlot0;

    // Run motor speeds (positive = intake, negative = eject)
    public static final double kRunSpeed = 1.0;
    public static final double kEjectSpeed = -0.1;

    // Calibration: slow retract until current spike or timeout
    public static final double kCalibrateSpeed = -0.2;
    public static final double kCalibrateCurrentThreshold = 35.0; // amps
    public static final double kCalibrateTimeoutSeconds = 3.0;
  }

  public static final class ShooterConstants {
    // Shooter tuning
    public static double targetRpm   = 1500.0;
    public static double readyRpm    = targetRpm * 0.98;   // Minimum RPM before feeder activates
    public static double feederSpeed = 0.9;      // Feeder open-loop duty cycle [0, 1]
    public static double maxOutput   = 0.85;     // Maximum closed-loop output [0, 1]

    public static final double kMinimumDistanceMeters = 1.0;
    public static final double kMaximumDistanceMeters = 3.0;
    public static final double kMinimumTargetRpm = 1500.0;
    public static final double kMaximumTargetRpm = 3800.0;

    // Closed-loop velocity PID gains for flywheel motor 1.
    public static double kP1 = 0.003;
    public static double kI1 = 0.0000;
    public static double kD1 = 0.004;
    public static double kF1 = 0.000162;

    // Closed-loop velocity PID gains for flywheel motor 2.
    public static double kP2 = 0.003;
    public static double kI2 = 0.00000;
    public static double kD2 = 0.004;
    public static double kF2 = 0.000162;

    public static final int FEEDER_CAN_ID = 21;
    public static final int SHOOTER1_CAN_ID = 22;
    public static final int SHOOTER2_CAN_ID = 23;
  }
}