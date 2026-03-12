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
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // ── Operator Interface ────────────────────────────────────────────────────
    public static final class OIConstants {
        public static final int kMainControllerPort   = 0;
        public static final int kBackupControllerPort = 1;

        public enum ControllerType { PS5, XBOX }
        public static final ControllerType kDriverControllerType = ControllerType.PS5;

        public static final double kDriveDeadband = 0.05;
        public static final double kTurnDeadband  = 0.12;
        public static final double kTwistDeadband = 0.5;

        public static final boolean kFieldRelative = true;
        public static final boolean kRateLimited   = true;
    }

    // ── Drive Constants ───────────────────────────────────────────────────────
    public static final class DriveConstants {
        // Maximum allowed speeds (not hardware limits)
        public static final double maxSpeedNormal = 3.3;                 // m/s
        public static final double maxAngularSpeed = 2.0 * Math.PI;     // rad/s

        // Slew rate limits
        public static final double kDirectionSlewRate  = 3.0;    // rad/s
        public static final double kMagnitudeSlewRate  = 3.5;    // percent/s (1 = 100%)
        public static final double kRotationalSlewRate = 2.0;    // percent/s (1 = 100%)

        // Chassis dimensions
        public static final double kTrackWidth = Units.inchesToMeters(24.5);
        public static final double kWheelBase  = Units.inchesToMeters(24.5);

        // Swerve drive kinematics
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d( kWheelBase / 2,  kTrackWidth / 2),   // front-left
                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),   // front-right
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),   // rear-left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));  // rear-right

        // Module angular offsets (radians)
        public static final double kFrontLeftChassisAngularOffset  = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset =  0;
        public static final double kBackLeftChassisAngularOffset   =  Math.PI;
        public static final double kBackRightChassisAngularOffset  =  Math.PI / 2;

        // SPARK MAX CAN IDs — driving motors
        public static final int kFrontLeftDrivingCanId  = 12;
        public static final int kRearLeftDrivingCanId   = 10;
        public static final int kFrontRightDrivingCanId = 14;
        public static final int kRearRightDrivingCanId  = 16;

        // SPARK MAX CAN IDs — turning motors
        public static final int kFrontLeftTurningCanId  = 13;
        public static final int kRearLeftTurningCanId   = 11;
        public static final int kFrontRightTurningCanId = 15;
        public static final int kRearRightTurningCanId  = 17;

        public static final boolean kGyroReversed = true;
    }

    // ── Vision Constants ──────────────────────────────────────────────────────
    /**
     * Vision and PhotonVision constants for 2026 Rebuilt.
     *
     * <p>Supports up to 4 cameras. Each camera can be individually enabled or
     * disabled via {@link #kCamerasEnabled}. Pose estimates from all enabled
     * cameras are fused into the drivetrain Kalman filter.
     */
    public static final class VisionConstants {
        // Number of cameras configured
        public static final int kNumCameras = 4;

        // Per-camera enable flags (index matches kCameraNames / kRobotToCams)
        public static final boolean[] kCamerasEnabled = {
            true,   // Left_Camera
            true,   // Front_Camera
            true,   // Rear_Left_Camera
            true,   // Rear_Right_Camera
        };

        // PhotonVision camera names (must match what is set in PhotonVision UI)
        public static final String[] kCameraNames = {
            "Left_Camera",
            "Front_Camera",
            "Rear_Left_Camera",
            "Rear_Right_Camera",
        };

        // Robot-centre → camera transforms (metres / radians)
        public static final Transform3d[] kRobotToCams = {
            // Left camera — mounted on the left side
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(5.00),
                    Units.inchesToMeters(11.5),
                    Units.inchesToMeters(32)),
                new Rotation3d(0, 0, Math.PI / 2)),

            // Front camera — mounted at the front centre
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(6.5),
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(36)),
                new Rotation3d(0, 0, Math.toRadians(0))),

            // Rear-left camera
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(4.25),
                    Units.inchesToMeters(36)),
                new Rotation3d(0, 0, Math.toRadians(180))),

            // Rear-right camera
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(-4),
                    Units.inchesToMeters(36)),
                new Rotation3d(0, 0, Math.toRadians(180))),
        };

        // Pose estimation tuning
        public static final double            kMaxAmbiguity      = 0.2;
        public static final Matrix<N3, N1>    kSingleTagStdDevs  = VecBuilder.fill(0.35, 0.35, 0.45);
        public static final Matrix<N3, N1>    kMultiTagStdDevs   = VecBuilder.fill(0.1,  0.1,  0.15);
        public static final double            kDistanceWeight    = 0.01;

        /**
         * Maximum distance (metres) at which a vision pose update is accepted.
         * Detections farther than this are discarded to avoid noisy measurements.
         */
        public static final double kMaxTargetDistance = 3.0;

        // Allowed positional error tolerances
        public static final double allowedXError = 0.025; // metres
        public static final double allowedYError = 0.025; // metres

        // PID for X-axis AprilTag alignment
        public static double kAlignP = 0.6;
        public static double kAlignI = 0.0;
        public static double kAlignD = 0.0;

        // PID for rotational alignment / heading lock
        public static double kRotP = 0.018;
        public static double kRotI = 0.00001;
        public static double kRotD = 0.0;
    }

    // ── Swerve Module Constants ───────────────────────────────────────────────
    public static final class ModuleConstants {
        public static final int     kDrivingMotorPinionTeeth  = 12;
        public static final boolean kTurningEncoderInverted   = true;

        public static final double kDrivingMotorFreeSpeedRps =
            NeoMotorConstants.kFreeSpeedRpm / 60.0;
        public static final double kWheelDiameterMeters      = 0.0746;
        public static final double kWheelCircumferenceMeters =
            kWheelDiameterMeters * Math.PI;
        public static final double kDrivingMotorReduction    =
            (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps   =
            (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor =
            (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;           // metres
        public static final double kDrivingEncoderVelocityFactor =
            ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;  // m/s

        public static final double kTurningEncoderPositionFactor = 2 * Math.PI;         // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // rad/s

        public static final double kTurningEncoderPositionPIDMinInput = 0;
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;

        public static final double kDrivingP  = 0.07;
        public static final double kDrivingI  = 0;
        public static final double kDrivingD  = 0.01;
        public static final double kDrivingFF = 0.25;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput =  1;

        public static final double kTurningP  = 0.65;
        public static final double kTurningI  = 0;
        public static final double kTurningD  = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput =  1;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    // ── NEO Motor Constants ───────────────────────────────────────────────────
    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    // ── PathPlanner Auto Constants ────────────────────────────────────────────
    public static final class AutoConstants {
        public static final PIDConstants translationConstants = new PIDConstants(2, 0.1, 0);
        public static final PIDConstants rotationConstants    = new PIDConstants(1, 0,   0);
    }

    // ── Navigation Constants ──────────────────────────────────────────────────
    public static final class NavigationConstants {
        /** PID gains for X/Y position control (field-relative). */
        public static final double kPositionP = 1.0;
        public static final double kPositionI = 0.00;
        public static final double kPositionD = 0.01;

        /** PID gains for rotation / heading control. */
        public static final double kRotationP = 0.025;
        public static final double kRotationI = 0.005;
        public static final double kRotationD = 0.0;

        public static final double kPositionTolerance = 0.05;  // metres
        public static final double kRotationTolerance = 5.0;   // degrees

        public static final double kMaxLinearSpeed  = 5.0; // m/s
        public static final double kMaxAngularSpeed = 1.0; // rad/s

        public static final double kFieldLength = 16.54; // metres
        public static final double kFieldWidth  = 8.21;  // metres
    }

    // ── Ball Tracking Constants ───────────────────────────────────────────────
    /**
     * PID constants for vision-based yaw alignment to balls.
     */
    public static final class BallTrackingConstants {
        public static final double kRotationP = 0.02;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.02;

        public static final double kYawTolerance     = 3.0; // degrees
        public static final double kMaxRotationSpeed = 0.3; // rad/s
    }

    // ── Addressable LED Strip Constants ───────────────────────────────────────
    public static final class LEDConstants {
        /** PWM port on the roboRIO where the LED data wire is connected. */
        public static final int kPort   = 9;

        /** Total number of individually addressable LEDs on the strip. */
        public static final int kLength = 30;

        /**
         * Flash toggle interval in seconds (0.25 s = 4 Hz).
         * Controls all flashing states: GYRO_FAULT, ENDGAME, MATCH_END, POSE_VALID fallback.
         */
        public static final double kFlashPeriod = 0.25;

        /**
         * Distance (metres) at which the POSE_VALID distance-gradient shows solid red.
         * Matches {@link VisionConstants#kMaxTargetDistance} so the gradient covers
         * exactly the range over which vision pose estimates are trusted.
         */
        public static final double kDistanceFarMeters   = 3.0;

        /**
         * Distance (metres) at which the POSE_VALID distance-gradient shows solid green.
         * Below this threshold the robot is considered "close" to the target.
         */
        public static final double kDistanceCloseMeters = 0.5;
    }

    // ── Ball Intake Constants ─────────────────────────────────────────────────
    /**
     * DISABLED — the physical intake hardware does not currently exist on the robot.
     * CAN ID 99 is intentionally invalid so that if the motor is ever accidentally
     * commanded, the SPARK MAX will not respond.  All speeds are 0 for the same reason.
     *
     * <p>To enable: update {@code kMotorCanId} with the real CAN ID, set the speed
     * constants to appropriate values, and un-comment the button bindings in
     * {@link RobotContainer}.
     */
    public static final class BallIntakeConstants {
        public static final int     kMotorCanId    = 99;    // INVALID — update before use
        public static final boolean kMotorInverted = false;
        public static final int     kCurrentLimit  = 25;    // amps
        public static final double  kIntakeSpeed   = 0.0;   // disabled
        public static final double  kEjectSpeed    = 0.0;   // disabled
        public static final double  kHoldSpeed     = 0.0;   // disabled
        public static final double  kSpeedDeadband = 0.05;
    }
}