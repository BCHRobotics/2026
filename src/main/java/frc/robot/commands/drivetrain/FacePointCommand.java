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
 */
public class FacePointCommand extends Command {
    private static final double kMinimumRadiusForDirection = 1e-6;

    private final Drivetrain m_drivetrain;
    private final double x;
    private final double y;
    private final double distance;

    private final DoubleSupplier joystickX;
    private final DoubleSupplier joystickY;
    
    // PID controllers for position and rotation control
    private final PIDController m_xController = new PIDController(
        NavigationConstants.kPositionP,
        NavigationConstants.kPositionI,
        NavigationConstants.kPositionD
    );
    private final PIDController m_yController = new PIDController(
        NavigationConstants.kPositionP,
        NavigationConstants.kPositionI,
        NavigationConstants.kPositionD
    );
    private final PIDController m_rotController = new PIDController(
        NavigationConstants.kRotationP,
        NavigationConstants.kRotationI,
        NavigationConstants.kRotationD
    );

    /**
     * Creates a new FacePointCommand.
     *
     * @param drivetrain The drivetrain subsystem
     * @param x Anchor point X coordinate in meters (field-relative)
     * @param y Anchor point Y coordinate in meters (field-relative)
     * @param distance Desired radial distance from the anchor point in meters
     */
    public FacePointCommand(Drivetrain drivetrain, DoubleSupplier joystickX, DoubleSupplier joystickY, double x, double y, double distance) {
        m_drivetrain = drivetrain;

        this.x = x;
        this.y = y;
        this.distance = distance;

        this.joystickX = joystickX;
        this.joystickY = joystickY;

        m_xController.setTolerance(NavigationConstants.kPositionTolerance);
        m_yController.setTolerance(NavigationConstants.kPositionTolerance);
        m_rotController.setTolerance(NavigationConstants.kRotationTolerance);
        m_rotController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_xController.reset();
        m_yController.reset();
        m_rotController.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getPose();

        double radialX = robotPose.getX() - x;
        double radialY = robotPose.getY() - y;
        double radius = Math.hypot(radialX, radialY);

        if (radius < kMinimumRadiusForDirection) {
            radialX = robotPose.getRotation().getCos();
            radialY = robotPose.getRotation().getSin();
            radius = 0.0;
        } else {
            radialX /= radius;
            radialY /= radius;
        }

        double targetX = x + (radialX * distance);
        double targetY = y + (radialY * distance);
        double targetHeadingDegrees = Math.toDegrees(Math.atan2(radialY, radialX));

        double xSpeed = m_xController.calculate(robotPose.getX(), targetX);
        double ySpeed = m_yController.calculate(robotPose.getY(), targetY);
        double rotation = m_rotController.calculate(robotPose.getRotation().getDegrees(), targetHeadingDegrees);

        xSpeed = MathUtil.clamp(xSpeed, -NavigationConstants.kMaxLinearSpeed, NavigationConstants.kMaxLinearSpeed);
        ySpeed = MathUtil.clamp(ySpeed, -NavigationConstants.kMaxLinearSpeed, NavigationConstants.kMaxLinearSpeed);
        rotation = MathUtil.clamp(rotation, -NavigationConstants.kMaxAngularSpeed, NavigationConstants.kMaxAngularSpeed);

        double rawJoystickX = MathUtil.applyDeadband(joystickX.getAsDouble(), OIConstants.kDriveDeadband);
        double rawJoystickY = MathUtil.applyDeadband(joystickY.getAsDouble(), OIConstants.kDriveDeadband);

        double tangentX = -radialY;
        double tangentY = radialX;
        double tangentialCommand = (rawJoystickX * tangentX) + (rawJoystickY * tangentY);

        double manualX = tangentX * tangentialCommand;
        double manualY = tangentY * tangentialCommand;

        m_drivetrain.drive(xSpeed + manualX, ySpeed + manualY, rotation, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0, 0, false, false);
        
        if (interrupted) {
            System.out.println("GoToPosition: Command interrupted");
        } else {
            System.out.println("GoToPosition: Arrived at target position");
            System.out.println("  Final pose: " + m_drivetrain.getPose());
        }
    }
}
