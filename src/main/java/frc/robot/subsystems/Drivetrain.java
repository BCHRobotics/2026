// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.Navx;
// Note: Navx.Port exists for USB-connected navX3 units; this robot uses CAN,
// so the Port enum is intentionally not imported/used here.
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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

  // ========================================================================
  // GYRO SETUP — two navigation sensors, one selector constant
  // ========================================================================
  // The robot has BOTH a navX2 (old, on the MXP port) and a navX3-CAN (new,
  // on the CAN bus) connected. Which one actually steers the robot is chosen
  // by DriveConstants.kGyroType in Constants.java — flip that constant to
  // switch sensors; no other code changes needed.
  //
  // WHY KEEP BOTH PLUGGED IN?
  //   * While testing the navX3 you can compare its angle against the navX2's
  //     on the dashboard (see "Gyro/NavX3 vs NavX2" in AdvantageScope) to
  //     prove the new sensor reads correctly BEFORE trusting it.
  //   * Both objects are created below, but only the selected one is read by
  //     getRotation2d() — the idle one costs almost nothing.
  //
  // IMPORTANT (hardware): the StudicaLib library that talks to the navX3-CAN
  // officially conflicts with the old Studica library used by the navX2.
  // We ship both vendordeps so this comparison setup works. If you ever see
  // strange gyro behavior after a WPILib/vendordep update, check
  // vendordeps/Studica.json + StudicaLib.json are still both present.
  // ========================================================================

  /** Old sensor: navX2 on the MXP port (SPI). Always created for comparison. */
  public final AHRS gyroNavX2 = new AHRS(NavXComType.kMXP_SPI);

  /**
   * New sensor: navX3-CAN on the CAN bus. Created only when selected —
   * constructing it opens a CAN session, so we don't open one we won't use.
   */
  public final Navx gyroNavX3 = DriveConstants.kGyroType == DriveConstants.GyroType.NAVX3_CAN
      ? new Navx(DriveConstants.kNavX3CanId)
      : null;

  /** Convenience accessor: whichever sensor kGyroType points at. */
  public Object getActiveGyro() {
    return DriveConstants.kGyroType == DriveConstants.GyroType.NAVX3_CAN ? gyroNavX3 : gyroNavX2;
  }

  // Slew rate filter variables for controlling lateral acceleration
  private double slew_currentRotation = 0.0;
  private double slew_currentTranslationDir = 0.0;
  private double slew_currentTranslationMag = 0.0;

  // A percentage value (0-1) for the linear speed of the robot
  private double maxSpeed = DriveConstants.maxSpeedNormal;

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
      resetActiveGyro(); // active sensor now treats current direction as zero
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

    setSpeedPercent();

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
    
    // Add gyro heading to Shuffleboard
    SmartDashboard.putNumber("Gyro Heading", getHeading());
    logAdvantageScopeData();
    logGyroComparison();

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
    
    System.out.print(diagnostics.toString());
  }

  /**
   * CHANGE (navX3): logs BOTH sensors' angles every cycle so they can be
   * compared on AdvantageScope while testing. When kGyroType = NAVX3_CAN,
   * watch "Gyro/NavX3vsNavX2/DeltaDegrees" — it should stay small (well
   * under 1 degree) during normal driving before you fully trust the new
   * sensor. The delta is also logged when the navX3 is NOT selected, so you
   * can validate it in advance without changing any driving behavior.
   */
  private void logGyroComparison() {
    double navX2AngleDeg = -gyroNavX2.getAngle(); // same sign convention as getRotation2d()
    double navX3AngleDeg =
        (gyroNavX3 != null) ? -gyroNavX3.getAngle().in(edu.wpi.first.units.Units.Degrees) : Double.NaN;

    Logger.recordOutput("Gyro/NavX2/AngleDegrees", navX2AngleDeg);
    Logger.recordOutput("Gyro/NavX3/AngleDegrees", navX3AngleDeg);
    Logger.recordOutput("Gyro/NavX3/Connected", gyroNavX3 != null);
    if (gyroNavX3 != null) {
      // inputModulus wraps the difference to [-180, 180] so crossing the
      // 0/360 boundary doesn't produce a huge fake spike.
      double delta = MathUtil.inputModulus(navX3AngleDeg - navX2AngleDeg, -180.0, 180.0);
      Logger.recordOutput("Gyro/NavX3vsNavX2/DeltaDegrees", delta);
    }
  }

  private void logAdvantageScopeData() {
    Pose2d fusedPose = getPose();
    Pose2d odometryPose = getOdometryPose();
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();

    Logger.recordOutput("Drivetrain/Pose", fusedPose);
    Logger.recordOutput("Drivetrain/Pose3d", new edu.wpi.first.math.geometry.Pose3d(fusedPose));
    Logger.recordOutput("Drivetrain/OdometryPose", odometryPose);
    Logger.recordOutput("Drivetrain/OdometryPose3d", new edu.wpi.first.math.geometry.Pose3d(odometryPose));
    Logger.recordOutput("Drivetrain/ModuleStates", getModuleStates());
    Logger.recordOutput("Drivetrain/HeadingDegrees", getHeading());
    Logger.recordOutput("Drivetrain/ChassisSpeeds/VxMetersPerSecond", chassisSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Drivetrain/ChassisSpeeds/VyMetersPerSecond", chassisSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Drivetrain/ChassisSpeeds/OmegaRadiansPerSecond", chassisSpeeds.omegaRadiansPerSecond);
    Logger.recordOutput(
        "Drivetrain/VisionPoseErrorMeters",
        fusedPose.getTranslation().getDistance(odometryPose.getTranslation())
    );
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
   * Resets (zeros) whichever gyro DriveConstants.kGyroType selects.
   *
   * CHANGE (navX3): the two sensors zero differently —
   *   navX2: reset()  (classic method, also restarts its calibration)
   *   navX3: resetYaw() (returns a status code; 0 usually means OK)
   * This helper hides that difference from the rest of the code.
   */
  private void resetActiveGyro() {
    if (DriveConstants.kGyroType == DriveConstants.GyroType.NAVX3_CAN) {
      int status = gyroNavX3.resetYaw();
      Logger.recordOutput("Gyro/NavX3/ResetStatus", status);
    } else {
      gyroNavX2.reset();
    }
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
    setSpeedPercent();
  }

  /** Disables turbo speed when released */
  public void disableTurboSpeed() {
    isTurboSpeed = false;
    setSpeedPercent();
  }

  /** Returns the robot heading as a Rotation2d. Always authoritative source.
   *
   * CHANGE (navX3): reads whichever sensor DriveConstants.kGyroType selects.
   * The navX2's getAngle() returns degrees directly; the navX3's getAngle()
   * returns a modern WPILib "Angle" measurement object, so we convert it to
   * plain degrees with .in(Degrees). Both are negated because the sensor is
   * mounted upside-down (kGyroReversed), same as before.
   */
  public Rotation2d getRotation2d() {
      if (DriveConstants.kGyroType == DriveConstants.GyroType.NAVX3_CAN) {
        // New sensor: navX3-CAN. getAngle() gives a continuous angle measure.
        return Rotation2d.fromDegrees(-gyroNavX3.getAngle().in(edu.wpi.first.units.Units.Degrees));
      }
      // Old sensor: navX2 via MXP SPI (unchanged behavior).
      return Rotation2d.fromDegrees(-gyroNavX2.getAngle());
  }

  /** Returns the robot heading in degrees. */
  public double getHeading() {
      return getRotation2d().getDegrees();
  }

  // Updated the max speed of the robot based on what mode is enabled
  public void setSpeedPercent() {
    if (isTurboSpeed) {
      maxSpeed = DriveConstants.maxSpeedTurbo;
    } else {
      maxSpeed = DriveConstants.maxSpeedNormal;
    }
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
}