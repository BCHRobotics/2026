package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.NavigationConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class ClimbCommand extends Command {
  // The command has two phases: first drive into the climb position, then run the motor
  // until the hard-stop/limit input says we have reached the end of travel.
  private enum Phase {
    DRIVING,
    CLIMBING
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

  private Phase phase = Phase.DRIVING;
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
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", Phase.DRIVING.name());

    addRequirements(drivetrain, climber);
  }

  @Override
  public void initialize() {
    Pose2d selectedPose = selectedStartPoseSupplier.get();
    startPose = selectedPose != null ? selectedPose : ClimbConstants.kBlueLeftStartPose;
    // transformBy() applies motion in the robot's local frame, so this moves the robot
    // 30 inches "forward" relative to the heading stored in the selected start pose.
    targetPose = startPose.transformBy(new Transform2d(
      ClimbConstants.kDriveForwardDistanceMeters,
        0.0,
        new Rotation2d()));

    phase = Phase.DRIVING;

    // Resetting pose here makes the command repeatable for testing as long as the robot is
    // physically placed at the chosen start location before pressing the dashboard button.
    drivetrain.resetPose(startPose);
    climber.stop();

    xController.reset();
    yController.reset();
    rotationController.reset();
    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    rotationController.setSetpoint(startPose.getRotation().getDegrees());

    SmartDashboard.putBoolean(DASHBOARD_KEY_PREFIX + "Running", true);
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "SelectedStartPose", poseToString(startPose));
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "TargetPose", poseToString(targetPose));
    SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", phase.name());
  }

  @Override
  public void execute() {
    if (phase == Phase.DRIVING) {
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

      // Only switch to the climb motor once translation and heading are both within tolerance.
      if (xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint()) {
        drivetrain.setChassisSpeeds(new ChassisSpeeds());
        phase = Phase.CLIMBING;
        SmartDashboard.putString(DASHBOARD_KEY_PREFIX + "Phase", phase.name());
      }
      return;
    }

    // In the climbing phase we simply run until the Spark MAX limit input is pressed.
    climber.runClimber();
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
    // The limit switch is the safety stop for this command.
    return phase == Phase.CLIMBING && climber.isLimitSwitchPressed();
  }

  private String poseToString(Pose2d pose) {
    return String.format("(%.3f, %.3f, %.1f deg)",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }
}