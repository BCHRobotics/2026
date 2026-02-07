package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NavigationConstants;
import frc.robot.subsystems.Drivetrain;
import frc.utils.dto.m.Vector2;

/**
 * Command to drive the robot to a specified position on the field.
 * Uses PID control to navigate to the target X, Y coordinates with a specified rotation.
 * 
 * This command is field-relative and uses the robot's odometry to determine position.
 */
public class FacePointCommand extends Command {
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
     * @param x Target X coordinate in meters (field-relative)
     * @param y Target Y coordinate in meters (field-relative)
     * @param distance Target rotation in degrees (field-relative)
     */
    public FacePointCommand(Drivetrain drivetrain, DoubleSupplier joystickX, DoubleSupplier joystickY, double x, double y, double distance) {
        m_drivetrain = drivetrain;

        this.x = x;
        this.y = y;
        this.distance = distance;

        this.joystickX = joystickX;
        this.joystickY = joystickY;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
        // System.out.println("GoToPosition: Navigating to position " + m_targetPose);
        // System.out.println("  Current pose: " + m_drivetrain.getPose());
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getPose();

        Vector2 pointToRobot = new Vector2(robotPose.getX() - x, robotPose.getY() - y);
        pointToRobot = Vector2.normalize(pointToRobot);
        pointToRobot = new Vector2(pointToRobot.x * distance, pointToRobot.y * distance);

        Vector2 closestPoint = new Vector2(x + pointToRobot.x, y + pointToRobot.y);

        // Set PID setpoints
        m_xController.setSetpoint(closestPoint.x);
        m_yController.setSetpoint(closestPoint.y);
        double desiredAngle = Math.atan2(-pointToRobot.y, -pointToRobot.x) / Math.PI * 180;
        double otherAngle = desiredAngle > 0 ? desiredAngle - 360 : desiredAngle + 360;

        double currentAngle = robotPose.getRotation().getDegrees();
        
        double angle = Math.abs(currentAngle - desiredAngle) < Math.abs(currentAngle - otherAngle) ? desiredAngle : otherAngle;

        m_rotController.setSetpoint(angle);
        
        // Calculate drive commands using PID
        double xSpeed = m_xController.calculate(robotPose.getX());
        double ySpeed = m_yController.calculate(robotPose.getY());
        double rotation = m_rotController.calculate(robotPose.getRotation().getDegrees());
        
        // Limit speeds for safety
        xSpeed = Math.max(-NavigationConstants.kMaxLinearSpeed, Math.min(NavigationConstants.kMaxLinearSpeed, xSpeed));
        ySpeed = Math.max(-NavigationConstants.kMaxLinearSpeed, Math.min(NavigationConstants.kMaxLinearSpeed, ySpeed));
        rotation = Math.max(-NavigationConstants.kMaxAngularSpeed, Math.min(NavigationConstants.kMaxAngularSpeed, rotation));

        Vector2 joystickInput = new Vector2(joystickX.getAsDouble(), joystickY.getAsDouble());
        Vector2 projectedJoystickInput = Vector2.project(joystickInput, new Vector2(x - robotPose.getX(), y - robotPose.getY()));

        joystickInput = Vector2.add(joystickInput, new Vector2(projectedJoystickInput.x * -1, projectedJoystickInput.y * -1));
        
        // Drive field-relative
        m_drivetrain.drive(xSpeed + joystickInput.x, ySpeed + joystickInput.y, rotation, true, false);
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
