package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.NavigationConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class ClimbCommand extends Command {
  private static final double kMinTranslationErrorForArrivalMeters = Units.inchesToMeters(1.0);

  // The command first drives from the robot's current estimated pose to the selected climb
  // start pose, extends the climber there if needed, and then advances to the final climb
  // approach point.
  private enum Phase {
    ALIGNING_TO_START,
    EXTENDING_AT_START,
    APPROACHING_CLIMB,
    CLIMBING,
    FINISHED
  }

  private static final String DASHBOARD_KEY_PREFIX = "ClimbCommand/";

  private final Drivetrain drivetrain;
  private final Climber climber;
  private final Supplier<Pose2d> selectedStartPoseSupplier;
  private final PIDController xController = new PIDController(
      NavigationConstants.kPositionP,
      NavigationConstants.kPositionI,
      NavigationConstants.kPositionD);
  private final PIDController yController = new PIDController(
      NavigationConstants.kPositionP,
      NavigationConstants.kPositionI,
      NavigationConstants.kPositionD);
  private final PIDController rotationController = new PIDController(
      NavigationConstants.kRotationP,
      NavigationConstants.kRotationI,
      NavigationConstants.kRotationD);

  private Phase phase = Phase.ALIGNING_TO_START;
  private Pose2d startPose = ClimbConstants.kBlueLeftStartPose;
  private Pose2d targetPose = ClimbConstants.kBlueLeftStartPose;

  public ClimbCommand(Drivetrain drivetrain, Climber climber, Supplier<Pose2d> selectedStartPoseSupplier) {
    this.drivetrain = drivetrain;
    this.climber = climber;
    this.selectedStartPoseSupplier = selectedStartPoseSupplier;

    xController.setTolerance(NavigationConstants.kPositionTolerance);
    yController.setTolerance(NavigationConstants.kPositionTolerance);
    rotationController.setTolerance(NavigationConstants.kRotationTolerance);
    rotationController.enableContinuousInput(-180.0, 180.0);

    SmartDashboard.putBoolean(DASHBOARD_KEY_PREFIX + "Running", false);
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", Phase.ALIGNING_TO_START.name());

    addRequirements(drivetrain, climber);
  }

  @Override
  public void initialize() {
    Pose2d selectedPose = selectedStartPoseSupplier.get();
    startPose = selectedPose != null ? selectedPose : ClimbConstants.kBlueLeftStartPose;
    Pose2d currentPose = drivetrain.getPose();

    // transformBy() applies motion in the robot's local frame, so this moves the robot
    // 30 inches "forward" relative to the heading stored in the selected start pose.
    targetPose = startPose.transformBy(new Transform2d(
      ClimbConstants.kDriveForwardDistanceMeters,
        0.0,
        new Rotation2d()));

    phase = Phase.ALIGNING_TO_START;
    climber.stop();

    xController.reset();
    yController.reset();
    rotationController.reset();
    setDriveTarget(startPose);

    SmartDashboard.putBoolean(DASHBOARD_KEY_PREFIX + "Running", true);
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "InitialPose", poseToString(currentPose));
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "SelectedStartPose", poseToString(startPose));
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "TargetPose", poseToString(targetPose));
    SmartDashboard.putBoolean(DASHBOARD_KEY_PREFIX + "AtStartPose", false);
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", phase.name());
  }

  @Override
  public void execute() {
    if (phase == Phase.ALIGNING_TO_START) {
      if (driveToPose(startPose)) {
        drivetrain.setChassisSpeeds(new ChassisSpeeds());
        if (climber.isExtendLimitReached()) {
          phase = Phase.APPROACHING_CLIMB;
        } else {
          phase = Phase.EXTENDING_AT_START;
        }

        SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", phase.name());
      }

      return;
    }

    if (phase == Phase.EXTENDING_AT_START) {
      drivetrain.setChassisSpeeds(new ChassisSpeeds());

      if (climber.isExtendLimitReached()) {
        climber.stop();
        phase = Phase.APPROACHING_CLIMB;
        SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", phase.name());
      } else {
        climber.extendClimber();
      }

      return;
    }

    if (phase == Phase.APPROACHING_CLIMB) {
      setDriveTarget(targetPose);

      if (driveToPose(targetPose)) {
        drivetrain.setChassisSpeeds(new ChassisSpeeds());
        phase = Phase.CLIMBING;
        SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", phase.name());
      }

      return;
    }

    if (phase == Phase.CLIMBING) {
      drivetrain.setChassisSpeeds(new ChassisSpeeds());

      if (climber.isRetractLimitReached()) {
        climber.stop();
        phase = Phase.FINISHED;
        SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", phase.name());
      } else {
        climber.retractClimber();
      }

      return;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(new ChassisSpeeds());
    climber.stop();

    SmartDashboard.putBoolean(DASHBOARD_KEY_PREFIX + "Running", false);
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "LastResult", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.FINISHED;
  }

  private void setDriveTarget(Pose2d pose) {
    xController.setSetpoint(pose.getX());
    yController.setSetpoint(pose.getY());
    rotationController.setSetpoint(pose.getRotation().getDegrees());
  }

  private boolean driveToPose(Pose2d desiredPose) {
    Pose2d currentPose = drivetrain.getPose();
    double xSpeed = xController.calculate(currentPose.getX());
    double ySpeed = yController.calculate(currentPose.getY());
    double rotationSpeed = rotationController.calculate(currentPose.getRotation().getDegrees());

    double translationMagnitude = Math.hypot(xSpeed, ySpeed);
    if (translationMagnitude > ClimbConstants.kDriveMaxSpeedMetersPerSecond && translationMagnitude > 1e-6) {
      double scale = ClimbConstants.kDriveMaxSpeedMetersPerSecond / translationMagnitude;
      xSpeed *= scale;
      ySpeed *= scale;
    }

    rotationSpeed = MathUtil.clamp(
        rotationSpeed,
        -NavigationConstants.kMaxAngularSpeed,
        NavigationConstants.kMaxAngularSpeed);

    // The PID controllers compute field-relative X/Y speeds. We convert those into the
    // robot-relative wheel commands that the swerve drivetrain actually needs.
    drivetrain.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed,
        ySpeed,
        rotationSpeed,
        Rotation2d.fromDegrees(drivetrain.getHeading())));

    SmartDashboard.putNumber(DASHBOARD_KEY_PREFIX + "CurrentPoseX", currentPose.getX());
    SmartDashboard.putNumber(DASHBOARD_KEY_PREFIX + "CurrentPoseY", currentPose.getY());
    SmartDashboard.putNumber(DASHBOARD_KEY_PREFIX + "CurrentHeadingDegrees", currentPose.getRotation().getDegrees());
    SmartDashboard.putBoolean(DASHBOARD_KEY_PREFIX + "AtStartPose", isAtPose(currentPose, startPose));

    return isAtPose(currentPose, desiredPose);
  }

  private boolean isAtPose(Pose2d currentPose, Pose2d desiredPose) {
    double translationError = currentPose.getTranslation().getDistance(desiredPose.getTranslation());
    double headingErrorDegrees = Math.abs(
        MathUtil.inputModulus(
            currentPose.getRotation().getDegrees() - desiredPose.getRotation().getDegrees(),
            -180.0,
            180.0));

    double translationTolerance = Math.max(
        NavigationConstants.kPositionTolerance,
        kMinTranslationErrorForArrivalMeters);

    return translationError <= translationTolerance
        && headingErrorDegrees <= NavigationConstants.kRotationTolerance;
  }

  private String poseToString(Pose2d pose) {
    return String.format("(%.3f, %.3f, %.1f deg)",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }
}