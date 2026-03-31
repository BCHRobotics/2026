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
    // If the robot is effectively at the hub center, the hub-to-robot direction is undefined.
    private static final double kMinimumDistanceForDirection = 1e-6;

    // Drivetrain is the only subsystem this command controls.
    private final Drivetrain drivetrain;
    // PID closes the loop on robot heading in degrees.
    private final PIDController rotationController = new PIDController(
        NavigationConstants.kRotationP,
        NavigationConstants.kRotationI,
        NavigationConstants.kRotationD
    );

    public PointRearToAllianceHubCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Heading wraps around, so -179 and 179 degrees should be treated as nearby angles.
        rotationController.setTolerance(NavigationConstants.kRotationTolerance);
        rotationController.enableContinuousInput(-180.0, 180.0);

        // This command owns the drivetrain while it is running.
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Clear any previous accumulated controller state before starting a new aim action.
        rotationController.reset();
    }

    @Override
    public void execute() {
        // Get the robot pose in field coordinates.
        Pose2d robotPose = drivetrain.getPose();
        // Pick the hub that belongs to the current alliance.
        Translation2d hubCenter = getAllianceHubCenter();

        // Build the field-relative vector from the hub to the robot.
        // Pointing the robot heading along this vector makes the robot's rear face the hub.
        double radialX = robotPose.getX() - hubCenter.getX();
        double radialY = robotPose.getY() - hubCenter.getY();

        // If the robot is essentially on top of the hub center, there is no stable direction
        // to derive from position alone. Fall back to the robot's current heading vector so
        // the controller holds its present orientation instead of snapping to an arbitrary angle.
        if (Math.hypot(radialX, radialY) < kMinimumDistanceForDirection) {
            radialX = robotPose.getRotation().getCos();
            radialY = robotPose.getRotation().getSin();
        }

        // Convert the hub-to-robot vector into the heading that should align the robot's rear
        // toward the hub.
        double targetHeadingDegrees = Math.toDegrees(Math.atan2(radialY, radialX));
        // Compute the angular velocity request needed to move from current heading to target.
        double rotationCommand = rotationController.calculate(
            robotPose.getRotation().getDegrees(),
            targetHeadingDegrees
        );

        // Limit the commanded rotation rate to the configured drivetrain maximum.
        double clampedRotation = MathUtil.clamp(
            rotationCommand,
            -NavigationConstants.kMaxAngularSpeed,
            NavigationConstants.kMaxAngularSpeed
        );

        // Rotate in place; translation stays zero while this command is active.
        drivetrain.drive(0.0, 0.0, clampedRotation, OIConstants.kFieldRelative, OIConstants.kRateLimited);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all rotational motion when the command finishes or is interrupted.
        drivetrain.drive(0.0, 0.0, 0.0, false, false);
    }

//TODO: VALIDATE THIS FUNCTION IS CORRECT FOR BOTH ALLIANCES
//I suspect this function is failing for some reason, resulting in the robot pointing to the wrong hub.


private Translation2d getAllianceHubCenter() {
        // Default to blue if the alliance is unavailable, which can happen while disconnected.
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red
            ? NavigationConstants.kRedAllianceHubCenter
            : NavigationConstants.kBlueAllianceHubCenter;
    }
}