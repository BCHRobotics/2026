package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 * Command to align robot to a specific AprilTag.
 * Uses PID control to drive toward the tag and rotate to face it.
 * 
 * This is an example/template - requires tuning and testing!
 */
public class AlignToAprilTagCommand extends Command {
    private final Vision m_vision;
    private final Drivetrain m_drivetrain;
    private final int m_targetTagId;
    
    // PID controllers for alignment
    private final PIDController m_xController = new PIDController(0.5, 0, 0);
    private final PIDController m_yController = new PIDController(0.5, 0, 0);
    private final PIDController m_rotController = new PIDController(0.1, 0, 0);
    
    private static final double kPositionTolerance = 0.1; // meters
    private static final double kRotationTolerance = 5.0; // degrees

    /**
     * Creates a new AlignToAprilTagCommand.
     *
     * @param vision The vision subsystem
     * @param drivetrain The drivetrain subsystem
     * @param targetTagId The AprilTag ID to align to
     */
    public AlignToAprilTagCommand(Vision vision, Drivetrain drivetrain, int targetTagId) {
        m_vision = vision;
        m_drivetrain = drivetrain;
        m_targetTagId = targetTagId;

        // Set tolerances
        m_xController.setTolerance(kPositionTolerance);
        m_yController.setTolerance(kPositionTolerance);
        m_rotController.setTolerance(kRotationTolerance);
        m_rotController.enableContinuousInput(-180, 180);

        addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        // Get target tag pose from field layout
        var tagPose = m_vision.getFieldLayout().getTagPose(m_targetTagId);
        
        if (tagPose.isPresent()) {
            var target = tagPose.get().toPose2d();
            m_xController.setSetpoint(target.getX());
            m_yController.setSetpoint(target.getY());
            m_rotController.setSetpoint(target.getRotation().getDegrees());
        } else {
            System.err.println("AlignToAprilTag: Tag " + m_targetTagId + " not found in field layout!");
        }
    }

    @Override
    public void execute() {
        var currentPose = m_drivetrain.getPose();
        
        // Calculate drive commands using PID
        double xSpeed = m_xController.calculate(currentPose.getX());
        double ySpeed = m_yController.calculate(currentPose.getY());
        double rotation = m_rotController.calculate(currentPose.getRotation().getDegrees());
        
        // Limit speeds for safety
        xSpeed = Math.max(-0.5, Math.min(0.5, xSpeed));
        ySpeed = Math.max(-0.5, Math.min(0.5, ySpeed));
        rotation = Math.max(-0.3, Math.min(0.3, rotation));
        
        // Drive field-relative
        m_drivetrain.drive(xSpeed, ySpeed, rotation, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        // Finish when all controllers are at setpoint
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               m_rotController.atSetpoint();
    }
}
