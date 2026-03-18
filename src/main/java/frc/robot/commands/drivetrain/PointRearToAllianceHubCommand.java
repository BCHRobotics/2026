package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Rotates the robot so the rear stays pointed at the current alliance hub.
 */
public class PointRearToAllianceHubCommand extends Command {
    private static final double kMinimumDistanceForDirection = 1e-6;

    private final Drivetrain drivetrain;
    private final PIDController rotationController = new PIDController(
        NavigationConstants.kRotationP,
        NavigationConstants.kRotationI,
        NavigationConstants.kRotationD
    );

    public PointRearToAllianceHubCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        rotationController.setTolerance(NavigationConstants.kRotationTolerance);
        rotationController.enableContinuousInput(-180.0, 180.0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationController.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Translation2d hubCenter = getAllianceHubCenter();

        double radialX = robotPose.getX() - hubCenter.getX();
        double radialY = robotPose.getY() - hubCenter.getY();

        if (Math.hypot(radialX, radialY) < kMinimumDistanceForDirection) {
            radialX = robotPose.getRotation().getCos();
            radialY = robotPose.getRotation().getSin();
        }

        double targetHeadingDegrees = Math.toDegrees(Math.atan2(radialY, radialX));
        double rotationCommand = rotationController.calculate(
            robotPose.getRotation().getDegrees(),
            targetHeadingDegrees
        );

        double clampedRotation = MathUtil.clamp(
            rotationCommand,
            -NavigationConstants.kMaxAngularSpeed,
            NavigationConstants.kMaxAngularSpeed
        );

        drivetrain.drive(0.0, 0.0, clampedRotation, OIConstants.kFieldRelative, OIConstants.kRateLimited);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0, 0.0, false, false);
    }

    private Translation2d getAllianceHubCenter() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red
            ? NavigationConstants.kRedAllianceHubCenter
            : NavigationConstants.kBlueAllianceHubCenter;
    }
}