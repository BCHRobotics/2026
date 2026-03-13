package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Holds the robot at a fixed radius from a point while facing away from it.
 *
 * Translation PID maintains the requested standoff distance along the current
 * radial line from the point through the robot. Driver translation input is
 * projected onto the tangent of that circle so the operator can slide around
 * the point without disturbing the radius lock.
 *
 * In simpler terms:
 * The robot picks an invisible circle around the target point, stays on that
 * circle, and keeps its front pointed away from the center of the circle.
 */
public class FacePointCommand extends Command {
    // If the robot is exactly on top of the target point, the radial direction is undefined.
    // This tiny value lets us detect that case and fall back to the robot's current heading.
    private static final double kMinimumRadiusForDirection = 1e-6;

    // The drivetrain is the subsystem that actually moves the robot.
    private final Drivetrain m_drivetrain;

    // The point we are orbiting around, measured in field coordinates.
    private final double x;
    private final double y;

    // How far away from that point we want the robot to stay.
    private final double distance;

    // Joystick inputs from the driver.
    // We keep these so the driver can slide around the circle while the PID holds the radius.
    private final DoubleSupplier joystickX;
    private final DoubleSupplier joystickY;
    
    // PID for radius.
    // This controller only cares about one thing: how far the robot is from the anchor point.
    // If the robot is too close, it pushes outward. If the robot is too far, it pulls inward.
    private final PIDController m_radiusController = new PIDController(
        NavigationConstants.kPositionP/4,
        0,
        0
    );

    // PID for robot heading.
    // This turns the robot so its front points away from the anchor point.
    private final PIDController m_rotController = new PIDController(
        NavigationConstants.kRotationP/16,
        0,
        0
    );

    /**
     * Creates a new FacePointCommand.
     *
     * @param drivetrain The drivetrain subsystem
      * @param joystickX Driver translation input on the field X axis
      * @param joystickY Driver translation input on the field Y axis
     * @param x Anchor point X coordinate in meters (field-relative)
     * @param y Anchor point Y coordinate in meters (field-relative)
     * @param distance Desired radial distance from the anchor point in meters
     */
    public FacePointCommand(Drivetrain drivetrain, DoubleSupplier joystickX, DoubleSupplier joystickY, double x, double y, double distance) {
          // Save constructor inputs so the command can use them later.
        m_drivetrain = drivetrain;

        this.x = x;
        this.y = y;
        this.distance = distance;

        this.joystickX = joystickX;
        this.joystickY = joystickY;

        // Tell each PID controller how close is "close enough."
        m_radiusController.setTolerance(NavigationConstants.kPositionTolerance);
        m_rotController.setTolerance(NavigationConstants.kRotationTolerance);

        // Heading wraps around, so -179 degrees and 179 degrees are almost the same.
        // Continuous input makes the PID choose the shorter turn direction.
        m_rotController.enableContinuousInput(-180, 180);

        // This command needs exclusive control of the drivetrain while it runs.
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Clear any leftover accumulated state from previous runs of this command.
        m_radiusController.reset();
        m_rotController.reset();
    }

    @Override
    public void execute() {
        // Read the robot's current position and heading from odometry.
        Pose2d robotPose = m_drivetrain.getPose();

        // Build the vector from the anchor point to the robot.
        // This tells us which direction "outward" currently is.
        double radialX = robotPose.getX() - x;
        double radialY = robotPose.getY() - y;

        // The radius is the straight-line distance from the anchor point to the robot.
        double radius = Math.hypot(radialX, radialY);

        if (radius < kMinimumRadiusForDirection) {
            // If the robot is basically on top of the point, there is no clear outward direction.
            // In that special case, use the robot's current heading as the outward direction.
            radialX = robotPose.getRotation().getCos();
            radialY = robotPose.getRotation().getSin();
            radius = 0.0;
        } else {
            // Normalize the radial vector so it has length 1.
            // After this, radialX and radialY represent direction only, not distance.
            radialX /= radius;
            radialY /= radius;
        }

        // Convert the outward direction into a heading angle in degrees.
        // Because the heading points along the radial vector, the robot's front points away from the anchor point.
        double targetHeadingDegrees = Math.toDegrees(Math.atan2(radialY, radialX));

        // The radius PID compares the current radius to the desired radius.
        // Its output is a single speed along the radial direction.
        double radialSpeed = m_radiusController.calculate(radius, distance);

        // Convert that one radial speed back into field X/Y components.
        double xSpeed = radialX * radialSpeed;
        double ySpeed = radialY * radialSpeed;

        // The rotation PID is still independent and only worries about heading.
        double rotation = m_rotController.calculate(robotPose.getRotation().getDegrees(), targetHeadingDegrees);

        // Clamp the outputs so the command cannot request speeds beyond the allowed limits.
        xSpeed = MathUtil.clamp(xSpeed, -NavigationConstants.kMaxLinearSpeed, NavigationConstants.kMaxLinearSpeed);
        ySpeed = MathUtil.clamp(ySpeed, -NavigationConstants.kMaxLinearSpeed, NavigationConstants.kMaxLinearSpeed);
        rotation = MathUtil.clamp(rotation, -NavigationConstants.kMaxAngularSpeed, NavigationConstants.kMaxAngularSpeed);

        // Apply joystick deadband so tiny stick noise does not make the robot drift.
        double rawJoystickX = MathUtil.applyDeadband(joystickX.getAsDouble(), OIConstants.kDriveDeadband);
        double rawJoystickY = MathUtil.applyDeadband(joystickY.getAsDouble(), OIConstants.kDriveDeadband);

        // A tangent vector is perpendicular to the radial vector.
        // Moving along this tangent slides the robot around the circle without moving closer or farther away.
        double tangentX = -radialY;
        double tangentY = radialX;

        // Project the driver's joystick request onto the tangent direction.
        // This keeps only the part of the joystick motion that moves around the circle.
        double tangentialCommand = (rawJoystickX * tangentX) + (rawJoystickY * tangentY);

        // Convert that 1D tangent command back into field X/Y components.
        double manualX = tangentX * tangentialCommand;
        double manualY = tangentY * tangentialCommand;

        // Combine automatic PID correction with the driver's tangential motion,
        // then command the drivetrain in field-relative coordinates.
        m_drivetrain.drive(xSpeed + manualX, ySpeed + manualY, rotation, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the drivetrain when the command ends.
        m_drivetrain.drive(0, 0, 0, false, false);
        
        if (interrupted) {
            // This branch runs if something else cancels the command.
            System.out.println("GoToPosition: Command interrupted");
        } else {
            // This branch runs if the command ends normally.
            System.out.println("GoToPosition: Arrived at target position");
            System.out.println("  Final pose: " + m_drivetrain.getPose());
        }
    }
}
