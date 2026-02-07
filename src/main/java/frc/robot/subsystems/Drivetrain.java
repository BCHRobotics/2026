// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  // front left wheel
  private final MAXSwerveModule frontLeftModule = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);
  // front right wheel
  private final MAXSwerveModule frontRightModule = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);
  // rear left wheel
  private final MAXSwerveModule rearLeftModule = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);
  // rear right wheel
  private final MAXSwerveModule rearRightModule = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // Slew rate filter variables for controlling lateral acceleration
  private double slew_currentRotation = 0.0;
  private double slew_currentTranslationDir = 0.0;
  private double slew_currentTranslationMag = 0.0;

  // A percentage value (0-1) for the linear speed of the robot
  private double maxSpeed = 0.2;

  // slew rates (basically ramp rates?) for the swerve drive
  private SlewRateLimiter slew_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter slew_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double slew_prevTime = WPIUtilJNI.now() * 1e-6;

  // boolean for keeping track of robot alliance (used for flipping auto path)
  public boolean isRedAlliance;

  // Timer for pose output throttling (once per second)
  private double lastPrintTime = 0.0;
  
  // Optional reference to Vision subsystem for diagnostics
  private Vision vision = null;

  // Odometry class for tracking robot pose (basic wheel odometry)
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      });
  
  /**
   * Pose estimator that fuses wheel odometry with vision measurements using Kalman filtering.
   * 
   * Uses an Unscented Kalman Filter (UKF) to optimally combine:
   * - Wheel odometry (continuous, but drifts over time due to wheel slip)
   * - Vision measurements (accurate but intermittent, may have outliers)
   * 
   * How it works:
   * 1. Predicts pose based on wheel odometry (process model)
   * 2. Corrects prediction when vision measurements arrive (measurement update)
   * 3. Weights each measurement by its uncertainty (standard deviations)
   * 4. Produces optimal estimate that's better than either source alone
   * 
   * Benefits over basic odometry:
   * - Corrects for wheel slip and drift using AprilTag vision
   * - Smoothly integrates intermittent vision measurements
   * - Handles measurement noise and outliers gracefully
   * - Provides statistically optimal pose estimate
   * 
   * The vision subsystem calls addVisionMeasurement() to provide vision updates.
   */
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      },
      new Pose2d()
  );

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    
  }
  
  /**
   * Sets the Vision subsystem reference for diagnostic output.
   * 
   * @param vision The Vision subsystem
   */
  public void setVision(Vision vision) {
    this.vision = vision;
  }

  @Override
  public void periodic() {
    // Update odometry with latest wheel positions
    odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        });
    
    // Update pose estimator with latest wheel positions
    // Vision measurements are added separately via addVisionMeasurement()
    poseEstimator.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        });

    // Print comprehensive diagnostics once per second
    double currentTime = WPIUtilJNI.now() * 1e-6;
    if (currentTime - lastPrintTime >= 1.0) {
      printDiagnostics();
      lastPrintTime = currentTime;
    }
  }
  

  // TODO: remove this
  /**
   * Prints comprehensive diagnostic information to console.
   * Includes robot position, heading, visible AprilTags, and motor currents.
   */
  private void printDiagnostics() {
    StringBuilder diagnostics = new StringBuilder();
    diagnostics.append("\n========== ROBOT DIAGNOSTICS ==========\n");
    
    // Robot Position and Heading
    Pose2d pose = getPose();
    diagnostics.append(String.format("Position: X=%.2fm, Y=%.2fm, Heading=%.1f°\n", 
        pose.getX(), pose.getY(), getHeading()));
    
    // Navigation to specific AprilTags
    if (vision != null) {
      Vision.TagNavigationInfo tag3 = vision.getTagNavigationInfo(3);
      Vision.TagNavigationInfo tag4 = vision.getTagNavigationInfo(4);
      
      diagnostics.append("Target AprilTags:\n");
      if (tag3.tagExists) {
        diagnostics.append(String.format("  Tag 3: Distance=%.2fm, Heading=%.1f°\n",
            tag3.distance, tag3.heading));
      } else {
        diagnostics.append("  Tag 3: NOT IN FIELD LAYOUT\n");
      }
      
      if (tag4.tagExists) {
        diagnostics.append(String.format("  Tag 4: Distance=%.2fm, Heading=%.1f°\n",
            tag4.distance, tag4.heading));
      } else {
        diagnostics.append("  Tag 4: NOT IN FIELD LAYOUT\n");
      }
    }
    
    // Vision - AprilTags with detailed information
    if (vision != null) {
      java.util.List<Vision.AprilTagInfo> tagInfos = vision.getDetailedAprilTagInfo();
      diagnostics.append(String.format("AprilTags Visible: %d\n", tagInfos.size()));
      
      if (!tagInfos.isEmpty()) {
        int updatingCount = 0;
        for (Vision.AprilTagInfo info : tagInfos) {
          if (info.usedForPoseUpdate) updatingCount++;
        }
        diagnostics.append(String.format("  (%d updating pose, %d rejected due to high ambiguity)\n", 
            updatingCount, tagInfos.size() - updatingCount));
        
        for (Vision.AprilTagInfo info : tagInfos) {
          String updateStatus = info.usedForPoseUpdate ? "UPDATING" : "REJECTED";
          diagnostics.append(String.format("  Tag %2d: Ambiguity=%.3f, Dist=%.2fm, Cam=%s [%s]\n",
              info.id, info.ambiguity, info.distance, info.cameraName, updateStatus));
        }
      } else {
        diagnostics.append("  No tags detected\n");
      }
    } else {
      diagnostics.append("AprilTags: Vision subsystem not initialized\n");
    }
    
    diagnostics.append("=======================================\n");
    
    System.out.print(diagnostics.toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * 
   * This returns the pose from the pose estimator, which fuses wheel odometry
   * with vision measurements for improved accuracy.
   * 
   * @return The pose (uses vision fusion if available, otherwise wheel odometry)
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  
  /**
   * Returns the wheel-odometry-only pose (no vision fusion).
   * 
   * Useful for debugging or comparing odometry vs. vision-fused estimates.
   * 
   * @return The odometry-only pose
   */
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   * 
   * KEEP IN MIND this doesn't actually set the gyro,
   * the odometry just works with the current heading as an offset.
   * 
   * This resets both the basic odometry and the vision-fused pose estimator.
   * 
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        },
        pose);
    
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        },
        pose);
  }
  
  /**
   * Adds a vision measurement to the Kalman filter pose estimator.
   * 
   * This is called by the Vision subsystem when it has a new AprilTag-based
   * pose estimate. The Unscented Kalman Filter (UKF) will optimally fuse this
   * with wheel odometry based on the provided standard deviations.
   * 
   * Kalman Filter Operation:
   * - Standard deviations represent measurement uncertainty (covariance)
   * - Lower stddev = filter trusts this measurement more (higher Kalman gain)
   * - Higher stddev = filter trusts odometry more (lower Kalman gain)
   * - Filter finds optimal balance between the two sources
   * 
   * Why timestamps matter:
   * - Vision has processing latency (~20-100ms)
   * - Timestamp allows filter to apply measurement to correct historical state
   * - Filter "rewinds" to that time, applies update, then "fast-forwards"
   * 
   * @param visionPose The vision-estimated robot pose
   * @param timestamp The timestamp of the vision measurement (from PhotonVision)
   * @param stdDevs Standard deviations (measurement uncertainty) [x, y, theta]
   *                Higher values = less trust in this measurement
   *                Format: [x_meters, y_meters, theta_radians]
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
  }

  /**
   * set the alliance to either BLUE SIDE or RED SIDE
   * @param isRed if true set to RED, if false set to BLUE
   */
  public void setAlliance(boolean isRed) {
    isRedAlliance = isRed;
  }

  /**
   * get whether the robot is on the RED SIDE or BLUE SIDE
   * @return is the robot on the RED SIDE? (if true red, if false blue)
   */
  public boolean getAlliance() {
    return isRedAlliance;
  }
  
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param maxSpeed      A 0-1 multiplier for the x and y speed of the robot.
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (slew_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / slew_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - slew_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, slew_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        slew_currentTranslationDir = SwerveUtils.StepTowardsCircular(slew_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        slew_currentTranslationMag = slew_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (slew_currentTranslationMag > 1e-4) { // Some small number to avoid floating-point errors with equality checking
          // Keep currentTranslationDir unchanged
          slew_currentTranslationMag = slew_magLimiter.calculate(0.0);
        } else {
          slew_currentTranslationDir = SwerveUtils.WrapAngle(slew_currentTranslationDir + Math.PI);
          slew_currentTranslationMag = slew_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        slew_currentTranslationDir = SwerveUtils.StepTowardsCircular(slew_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        slew_currentTranslationMag = slew_magLimiter.calculate(0.0);
      }
      slew_prevTime = currentTime;

      xSpeedCommanded = slew_currentTranslationMag * Math.cos(slew_currentTranslationDir);
      ySpeedCommanded = slew_currentTranslationMag * Math.sin(slew_currentTranslationDir);
      slew_currentRotation = slew_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      slew_currentRotation = rot;
    }

    /*
     * Convert the commanded speeds into the correct units for the drivetrain,
     * using the given max speed
     */
    double xSpeedDelivered = xSpeedCommanded * maxSpeed;
    double ySpeedDelivered = ySpeedCommanded * maxSpeed;
    double rotDelivered = slew_currentRotation * DriveConstants.maxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
 
    this.setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the swerve ModuleStates
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxSpeed);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    rearLeftModule.setDesiredState(desiredStates[2]);
    rearRightModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Gets the swerve ModuleStates.
   * @return The current SwerveModule states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(),
      frontRightModule.getState(),
      rearLeftModule.getState(),
      rearRightModule.getState()
    };
  }

  /**
   * reset the wheel encoders
   * in theory this should just reset the odometry,
   * BUT DO NOT CALL THIS FUNCTION USE THE resetPose() FUNCTION INSTEAD
   */
  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    rearLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  /**
   * tell the gyro to treat the current direction as zero
   * this will make the robot treat how its facing as field forward
   */
  public void zeroHeading() {
    gyro.reset();
    // Change this after
    resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from -Infinity to Infinity
   */
  public double getHeading() {
    // I'm multiplying the navx heading by -1 
    // because WPILib uses CCW as the positive direction
    // and NavX uses CW as the positive direction
    return Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees();
  }

  /**
   * Updated the max speed of the robot based on what mode is enabled
   */
  public void setSpeedPercent() {
    maxSpeed = DriveConstants.maxSpeedNormal;
  }   
  
  /**
   * Sets the speed of the robot chassis
   * @param speed The new chassis speed
   */
  public void setChassisSpeeds(ChassisSpeeds speed, DriveFeedforwards ff) {
    this.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speed));
  }

  /**
   * Gets the current speed of the robot chassis
   * @return The current chassis speed, as a ChassisSpeeds class
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(this.getModuleStates());
  }
}
