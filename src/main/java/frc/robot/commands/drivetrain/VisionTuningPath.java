package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.VisionTuningConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives a fixed out-and-back line for vision repeatability testing.
 * The command captures the current heading on initialize and holds that heading
 * while driving between fixed field positions.
 */
public class VisionTuningPath extends Command {
    private static final Pose2d[] WAYPOINTS = {
        new Pose2d(14.2, 1.0, new Rotation2d()),
        new Pose2d(14.2, 7.0, new Rotation2d()),
        new Pose2d(14.2, 1.0, new Rotation2d())
    };

    private final Drivetrain drivetrain;
    private final PIDController xController = new PIDController(
        NavigationConstants.kPositionP,
        NavigationConstants.kPositionI,
        NavigationConstants.kPositionD
    );
    private final PIDController yController = new PIDController(
        NavigationConstants.kPositionP,
        NavigationConstants.kPositionI,
        NavigationConstants.kPositionD
    );
    private final PIDController rotationController = new PIDController(
        NavigationConstants.kRotationP,
        NavigationConstants.kRotationI,
        NavigationConstants.kRotationD
    );

    private int targetIndex;
    private double heldHeadingDegrees;

    public VisionTuningPath(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        xController.setTolerance(NavigationConstants.kPositionTolerance);
        yController.setTolerance(NavigationConstants.kPositionTolerance);
        rotationController.setTolerance(NavigationConstants.kRotationTolerance);
        rotationController.enableContinuousInput(-180.0, 180.0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetIndex = 0;
        heldHeadingDegrees = drivetrain.getHeading();
        xController.reset();
        yController.reset();
        rotationController.reset();
        rotationController.setSetpoint(heldHeadingDegrees);
        updateTargetSetpoints();
    }

    @Override
    public void execute() {
        advanceIfAtWaypoint();

        if (targetIndex >= WAYPOINTS.length) {
            drivetrain.setChassisSpeeds(new ChassisSpeeds());
            return;
        }

        Pose2d currentPose = drivetrain.getPose();

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double rotationSpeed = rotationController.calculate(drivetrain.getHeading());

        double maxLinearSpeed = VisionTuningConstants.kMaxSpeedMetersPerSecond;
        maxLinearSpeed = Math.max(0.0, maxLinearSpeed);

        double translationMagnitude = Math.hypot(xSpeed, ySpeed);
        if (translationMagnitude > maxLinearSpeed && translationMagnitude > 1e-6) {
            double scale = maxLinearSpeed / translationMagnitude;
            xSpeed *= scale;
            ySpeed *= scale;
        }

        rotationSpeed = MathUtil.clamp(
            rotationSpeed,
            -NavigationConstants.kMaxAngularSpeed,
            NavigationConstants.kMaxAngularSpeed
        );

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            rotationSpeed,
            Rotation2d.fromDegrees(drivetrain.getHeading())
        );
        drivetrain.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setChassisSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return targetIndex >= WAYPOINTS.length;
    }

    private void advanceIfAtWaypoint() {
        while (targetIndex < WAYPOINTS.length && xController.atSetpoint() && yController.atSetpoint()) {
            targetIndex++;
            if (targetIndex < WAYPOINTS.length) {
                updateTargetSetpoints();
            }
        }
    }

    private void updateTargetSetpoints() {
        Pose2d targetPose = WAYPOINTS[targetIndex];
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
    }
}