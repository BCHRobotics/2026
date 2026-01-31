package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NavigationConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to drive the robot to a specified position on the field.
 * Uses PID control to navigate to the target X, Y coordinates with a specified rotation.
 * 
 * This command is field-relative and uses the robot's odometry to determine position.
 */
public class GoToPositionCommand extends Command {
    private final Drivetrain m_drivetrain;
    private final Pose2d m_targetPose;
    
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
     * Creates a new GoToPositionCommand.
     *
     * @param drivetrain The drivetrain subsystem
     * @param x Target X coordinate in meters (field-relative)
     * @param y Target Y coordinate in meters (field-relative)
     * @param rotation Target rotation in degrees (field-relative)
     */
    public GoToPositionCommand(Drivetrain drivetrain, double x, double y, double rotation) {
        this(drivetrain, new Pose2d(x, y, Rotation2d.fromDegrees(rotation)));
    }

    /**
     * Creates a new GoToPositionCommand.
     *
     * @param drivetrain The drivetrain subsystem
     * @param targetPose Target pose on the field
     */
    public GoToPositionCommand(Drivetrain drivetrain, Pose2d targetPose) {
        m_drivetrain = drivetrain;
        m_targetPose = targetPose;

        // Set tolerances
        m_xController.setTolerance(NavigationConstants.kPositionTolerance);
        m_yController.setTolerance(NavigationConstants.kPositionTolerance);
        m_rotController.setTolerance(NavigationConstants.kRotationTolerance);
        m_rotController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Set PID setpoints
        m_xController.setSetpoint(m_targetPose.getX());
        m_yController.setSetpoint(m_targetPose.getY());
        m_rotController.setSetpoint(m_targetPose.getRotation().getDegrees());
        
        System.out.println("GoToPosition: Navigating to position " + m_targetPose);
        System.out.println("  Current pose: " + m_drivetrain.getPose());
    }

    @Override
    public void execute() {
        var currentPose = m_drivetrain.getPose();
        
        // Calculate drive commands using PID
        double xSpeed = m_xController.calculate(currentPose.getX());
        double ySpeed = m_yController.calculate(currentPose.getY());
        double rotation = m_rotController.calculate(currentPose.getRotation().getDegrees());
        
        // Limit speeds for safety
        xSpeed = Math.max(-NavigationConstants.kMaxLinearSpeed, Math.min(NavigationConstants.kMaxLinearSpeed, xSpeed));
        ySpeed = Math.max(-NavigationConstants.kMaxLinearSpeed, Math.min(NavigationConstants.kMaxLinearSpeed, ySpeed));
        rotation = Math.max(-NavigationConstants.kMaxAngularSpeed, Math.min(NavigationConstants.kMaxAngularSpeed, rotation));
        
        // Drive field-relative
        m_drivetrain.drive(xSpeed, ySpeed, rotation, true, false);
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

    /*@Override
    public boolean isFinished() {
        // Finish when all controllers are at setpoint
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               m_rotController.atSetpoint();
    }*/
}
