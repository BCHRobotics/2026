// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
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

  private boolean isTurboSpeed = false;

  // Timer for pose output throttling (once per second)
  private double lastPrintTime = 0.0;
  
  // Optional reference to Vision subsystem for diagnostics
  private Vision vision = null;

  private final Field2d fusedField = new Field2d();
  private final Field2d odometryField = new Field2d();
  private final Field2d visionField = new Field2d();

  // Odometry class for tracking robot pose (basic wheel odometry)
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getRotation2d(),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      }
  );
  
  /**
   * Pose estimator that fuses wheel odometry with vision measurements using Kalman filtering.
   * The vision subsystem calls addVisionMeasurement() to provide vision updates.
   */
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getRotation2d(),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      },
      new Pose2d()
  );

  /** Resets only the gyro heading while preserving current translation. */
  public void zeroHeadingOnly() {
      gyro.reset(); // NavX now treats current direction as zero
      // offset rotation back to preserve translation
      odometry.resetPosition(
          Rotation2d.fromDegrees(0.0),
          new SwerveModulePosition[] {
              frontLeftModule.getPosition(),
              frontRightModule.getPosition(),
              rearLeftModule.getPosition(),
              rearRightModule.getPosition()
          },
          odometry.getPoseMeters()
      );

      poseEstimator.resetPosition(
          Rotation2d.fromDegrees(0.0),
          new SwerveModulePosition[] {
              frontLeftModule.getPosition(),
              frontRightModule.getPosition(),
              rearLeftModule.getPosition(),
              rearRightModule.getPosition()
          },
          poseEstimator.getEstimatedPosition()
      );
  }

  public void resetPose(Pose2d pose) {
      odometry.resetPosition(getRotation2d(), 
          new SwerveModulePosition[] {
              frontLeftModule.getPosition(),
              frontRightModule.getPosition(),
              rearLeftModule.getPosition(),
              rearRightModule.getPosition()
          }, pose);

      poseEstimator.resetPosition(getRotation2d(),
          new SwerveModulePosition[] {
              frontLeftModule.getPosition(),
              frontRightModule.getPosition(),
              rearLeftModule.getPosition(),
              rearRightModule.getPosition()
          }, pose);
  }

  // ----- Updated zeroHeading binding (driver triangle) -----
  public void zeroHeading() {
      // Only reset rotation, preserve translation
      zeroHeadingOnly();
  }

  // Creates a new Drivetrain subsystem
  public Drivetrain() {
    configureAutoBuilder(AutoConstants.translationConstants, AutoConstants.rotationConstants);
    SmartDashboard.putData("Field/Fused", fusedField);
    SmartDashboard.putData("Field/Odometry", odometryField);
    SmartDashboard.putData("Field/Vision", visionField);
  }

  /**
   * Configures PathPlanner AutoBuilder with the provided PID constants.
   *
   * @param translationConstants PID for X/Y translation
   * @param rotationConstants PID for robot heading
   */
  public void configureAutoBuilder(PIDConstants translationConstants, PIDConstants rotationConstants) {
    try {
      // Load robot configuration from PathPlanner GUI settings
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder for holonomic (swerve) drive
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (called at auto start)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier (MUST be robot-relative)
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method to drive robot
        new PPHolonomicDriveController(
          translationConstants,
          rotationConstants
        ),
        config, // Robot configuration
        () -> {
          // Flip path for red alliance (origin stays on blue side)
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        this // Reference to this subsystem
      );
    } catch (Exception e) {
      // Handle exception - AutoBuilder will not be available
      System.err.println("Failed to configure PathPlanner AutoBuilder:");
      e.printStackTrace();
      System.err.println("Make sure you have created a robot configuration in PathPlanner GUI!");
    }
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
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        });
    
    // Update pose estimator with latest wheel positions
    // Vision measurements are added separately via addVisionMeasurement()
    poseEstimator.update(
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        });

      Pose2d fusedPose = getPose();
      SmartDashboard.putNumber("Pose/Fused/X", fusedPose.getX());
      SmartDashboard.putNumber("Pose/Fused/Y", fusedPose.getY());
      SmartDashboard.putNumber("Pose/Fused/HeadingDegrees", fusedPose.getRotation().getDegrees());
      fusedField.setRobotPose(fusedPose);

    Pose2d odometryPose = getOdometryPose();
    SmartDashboard.putNumber("Pose/Odometry/X", odometryPose.getX());
    SmartDashboard.putNumber("Pose/Odometry/Y", odometryPose.getY());
    SmartDashboard.putNumber("Pose/Odometry/HeadingDegrees", odometryPose.getRotation().getDegrees());
    odometryField.setRobotPose(odometryPose);

    if (vision != null) {
      var visionEstimate = vision.getEstimatedGlobalPose();
      SmartDashboard.putBoolean("Pose/Vision/Available", visionEstimate.isPresent());

      if (visionEstimate.isPresent()) {
        Pose2d visionPose = visionEstimate.get().estimatedPose.toPose2d();
        SmartDashboard.putNumber("Pose/Vision/X", visionPose.getX());
        SmartDashboard.putNumber("Pose/Vision/Y", visionPose.getY());
        SmartDashboard.putNumber("Pose/Vision/HeadingDegrees", visionPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Pose/Vision/Timestamp", visionEstimate.get().timestampSeconds);
        visionField.setRobotPose(visionPose);
      }
    } else {
      SmartDashboard.putBoolean("Pose/Vision/Available", false);
    }

    SmartDashboard.putString("CAN/DrivetrainStatus", getDrivetrainCanStatus());

    // Print comprehensive diagnostics once every 5 seconds
    double currentTime = WPIUtilJNI.now() * 1e-6;
    if (currentTime - lastPrintTime >= 5.0) {
      printDiagnostics();
      lastPrintTime = currentTime;
    }
  }
  

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
    SmartDashboard.putString("Diagnostics/Drivetrain", diagnostics.toString());
    System.out.print(diagnostics.toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
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
   * Useful for debugging or comparing odometry vs. vision-fused estimates.
   * 
   * @return The odometry-only pose
   */
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   * This resets both the basic odometry and the vision-fused pose estimator.
   * 
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetPose(pose);
  }
  
  /**
   * Adds a vision measurement to the Kalman filter pose estimator.
   * 
   * This is called by the Vision subsystem when it has a new AprilTag-based
   * pose estimate. The Unscented Kalman Filter (UKF) will optimally fuse this
   * with wheel odometry based on the provided standard deviations.
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
                getRotation2d())
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

  /** Enables turbo speed while held */
  public void enableTurboSpeed() {
    isTurboSpeed = true;
    maxSpeed = DriveConstants.maxSpeedTurbo;
  }

  /** Disables turbo speed when released */
  public void disableTurboSpeed() {
    isTurboSpeed = false;
    maxSpeed = DriveConstants.maxSpeedNormal;
  }

  /** Returns the robot heading as a Rotation2d. Always authoritative source. */
  public Rotation2d getRotation2d() {
      // NavX upside-down mounting already corrects for CCW
      return Rotation2d.fromDegrees(gyro.getAngle());
  }

  /** Returns the robot heading in degrees. */
  public double getHeading() {
      return getRotation2d().getDegrees();
  }

  // Updated the max speed of the robot based on what mode is enabled
  public void setSpeedPercent() {
    maxSpeed = DriveConstants.maxSpeedNormal;
  }   
  
  /**
   * Sets the speed of the robot chassis using ChassisSpeeds
   * This is used by PathPlanner for autonomous movement
   * @param speed The chassis speeds to apply
   */
  public void setChassisSpeeds(ChassisSpeeds speed) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
    setModuleStates(moduleStates);
  }

  /**
   * Gets the current speed of the robot chassis
   * @return The current chassis speed, as a ChassisSpeeds class
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(this.getModuleStates());
  }
  
  /**
   * Gets the current robot-relative chassis speeds.
   * 
   * This is the method used by PathPlanner AutoBuilder.
   * Returns speeds relative to the robot (not field-relative).
   * 
   * @return Current robot-relative ChassisSpeeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getChassisSpeeds();
  }
  
  /**
   * Drives the robot using robot-relative chassis speeds.
   * 
   * This is the method used by PathPlanner AutoBuilder.
   * Accepts speeds that are relative to the robot, not the field.
   * 
   * @param speeds Robot-relative ChassisSpeeds to apply
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    setChassisSpeeds(speeds);
  }

  private String getDrivetrainCanStatus() {
    StringBuilder errors = new StringBuilder();

    appendCanError(errors, "FrontLeftDriving", frontLeftModule.m_drivingSpark.getLastError());
    appendCanError(errors, "FrontLeftTurning", frontLeftModule.m_turningSpark.getLastError());
    appendCanError(errors, "FrontRightDriving", frontRightModule.m_drivingSpark.getLastError());
    appendCanError(errors, "FrontRightTurning", frontRightModule.m_turningSpark.getLastError());
    appendCanError(errors, "RearLeftDriving", rearLeftModule.m_drivingSpark.getLastError());
    appendCanError(errors, "RearLeftTurning", rearLeftModule.m_turningSpark.getLastError());
    appendCanError(errors, "RearRightDriving", rearRightModule.m_drivingSpark.getLastError());
    appendCanError(errors, "RearRightTurning", rearRightModule.m_turningSpark.getLastError());

    if (errors.length() == 0) {
      return "HEALTHY";
    }

    return errors.toString();
  }

  private void appendCanError(StringBuilder errors, String deviceName, REVLibError error) {
    if (error == REVLibError.kOk) {
      return;
    }

    if (errors.length() > 0) {
      errors.append(" | ");
    }
    errors.append(deviceName).append(": ").append(error.name());
  }
}