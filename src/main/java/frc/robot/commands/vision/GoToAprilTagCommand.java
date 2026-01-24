package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 * Command to drive robot to a specified distance in front of an AprilTag.
 * Uses PID control to navigate to the target position and face the tag.
 * 
 * The robot will position itself at the specified distance directly in front
 * of the AprilTag, facing toward the tag.
 */
public class GoToAprilTagCommand extends Command {
    private final Vision m_vision;
    private final Drivetrain m_drivetrain;
    private final int m_targetTagId;
    private final double m_distanceFromTag; // meters
    
    // PID controllers for alignment
    private final PIDController m_xController = new PIDController(0.5, 0, 0);
    private final PIDController m_yController = new PIDController(0.5, 0, 0);
    private final PIDController m_rotController = new PIDController(0.1, 0, 0);
    
    private static final double kPositionTolerance = 0.1; // meters
    private static final double kRotationTolerance = 5.0; // degrees
    
    private Pose2d m_targetPose;

    /**
     * Creates a new GoToAprilTagCommand.
     *
     * @param vision The vision subsystem
     * @param drivetrain The drivetrain subsystem
     * @param targetTagId The AprilTag ID to navigate to
     * @param distanceFromTag Distance in meters to stop in front of the tag
     */
    public GoToAprilTagCommand(Vision vision, Drivetrain drivetrain, int targetTagId, double distanceFromTag) {
        m_vision = vision;
        m_drivetrain = drivetrain;
        m_targetTagId = targetTagId;
        m_distanceFromTag = distanceFromTag;

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
            Pose2d tagPose2d = tagPose.get().toPose2d();
            
            // Calculate position 1 meter in front of the tag
            // The tag's rotation tells us which direction is "forward" from the tag
            // We want to be 1 meter back from the tag, facing it
            Rotation2d tagRotation = tagPose2d.getRotation();
            
            // Create a translation that is distanceFromTag meters away from the tag
            // in the direction opposite to where the tag is facing
            Translation2d offsetFromTag = new Translation2d(
                -m_distanceFromTag, // Negative to go in front (opposite of tag's facing direction)
                0                   // No lateral offset
            ).rotateBy(tagRotation);
            
            // Calculate target pose: tag position + offset, facing toward the tag
            m_targetPose = new Pose2d(
                tagPose2d.getTranslation().plus(offsetFromTag),
                tagRotation.plus(Rotation2d.fromDegrees(180)) // Face toward the tag (opposite of tag's rotation)
            );
            
            // Set PID setpoints
            m_xController.setSetpoint(m_targetPose.getX());
            m_yController.setSetpoint(m_targetPose.getY());
            m_rotController.setSetpoint(m_targetPose.getRotation().getDegrees());
            
            System.out.println("GoToAprilTag: Navigating to tag " + m_targetTagId + 
                             " at distance " + m_distanceFromTag + "m");
            System.out.println("  Tag pose: " + tagPose2d);
            System.out.println("  Target pose: " + m_targetPose);
        } else {
            System.err.println("GoToAprilTag: Tag " + m_targetTagId + " not found in field layout!");
            m_targetPose = null;
        }
    }

    @Override
    public void execute() {
        // Skip execution if no valid target pose
        if (m_targetPose == null) {
            return;
        }
        
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
        
        if (interrupted) {
            System.out.println("GoToAprilTag: Command interrupted");
        } else {
            System.out.println("GoToAprilTag: Arrived at target position");
        }
    }

    @Override
    public boolean isFinished() {
        // Can't finish if no valid target
        if (m_targetPose == null) {
            return true; // End immediately if tag not found
        }
        
        // Finish when all controllers are at setpoint
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               m_rotController.atSetpoint();
    }
}
