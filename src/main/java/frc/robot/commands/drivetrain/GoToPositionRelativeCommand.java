package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NavigationConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to drive the robot to a specified position on the field with alliance-relative coordinates.
 * Uses PID control to navigate to the target X, Y coordinates with a specified rotation.
 * 
 * Alliance-Relative Coordinates:
 * - For Blue Alliance: (0, 0) is at the blue alliance corner (standard FRC coordinates)
 * - For Red Alliance: (0, 0) is mirrored to the opposite diagonal corner
 *   - X coordinates are mirrored: red_x = FIELD_LENGTH - blue_x
 *   - Y coordinates are mirrored: red_y = FIELD_WIDTH - blue_y
 *   - Rotation is mirrored: red_rotation = 180 - blue_rotation
 * 
 * This allows autonomous routines to use the same coordinates regardless of alliance color.
 */
public class GoToPositionRelativeCommand extends Command {
    private final Drivetrain m_drivetrain;
    private final double m_relativeX;
    private final double m_relativeY;
    private final double m_relativeRotation;
    private final Alliance m_alliance;
    
    private Pose2d m_targetPose;
    
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
     * Creates a new GoToPositionRelativeCommand with alliance-relative coordinates.
     *
     * @param drivetrain The drivetrain subsystem
     * @param relativeX Alliance-relative X coordinate in meters
     * @param relativeY Alliance-relative Y coordinate in meters
     * @param relativeRotation Alliance-relative rotation in degrees
     * @param alliance Alliance color (Red or Blue)
     */
    public GoToPositionRelativeCommand(Drivetrain drivetrain, double relativeX, double relativeY, 
                                       double relativeRotation, Alliance alliance) {
        m_drivetrain = drivetrain;
        m_relativeX = relativeX;
        m_relativeY = relativeY;
        m_relativeRotation = relativeRotation;
        m_alliance = alliance;

        // Set tolerances
        m_xController.setTolerance(NavigationConstants.kPositionTolerance);
        m_yController.setTolerance(NavigationConstants.kPositionTolerance);
        m_rotController.setTolerance(NavigationConstants.kRotationTolerance);
        m_rotController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    /**
     * Creates a new GoToPositionRelativeCommand with alliance color from DriverStation.
     * 
     * Will automatically detect the alliance color from the Driver Station.
     * If alliance is not available, defaults to Blue alliance.
     *
     * @param drivetrain The drivetrain subsystem
     * @param relativeX Alliance-relative X coordinate in meters
     * @param relativeY Alliance-relative Y coordinate in meters
     * @param relativeRotation Alliance-relative rotation in degrees
     */
    public GoToPositionRelativeCommand(Drivetrain drivetrain, double relativeX, double relativeY, 
                                       double relativeRotation) {
        this(drivetrain, relativeX, relativeY, relativeRotation, 
             DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    /**
     * Converts alliance-relative coordinates to field-absolute coordinates.
     * 
     * Blue Alliance: No conversion needed (0,0 is already at blue corner)
     * Red Alliance: Mirror across field diagonal
     *   - X: field_length - relative_x
     *   - Y: field_width - relative_y
     *   - Rotation: 180 - relative_rotation (facing opposite direction)
     * 
     * @return Field-absolute Pose2d
     */
    private Pose2d convertToFieldCoordinates() {
        double fieldX, fieldY, fieldRotation;
        
        if (m_alliance == Alliance.Red) {
            // Mirror for red alliance
            fieldX = NavigationConstants.kFieldLength - m_relativeX;
            fieldY = NavigationConstants.kFieldWidth - m_relativeY;
            fieldRotation = 180.0 - m_relativeRotation;
        } else {
            // Blue alliance uses coordinates as-is
            fieldX = m_relativeX;
            fieldY = m_relativeY;
            fieldRotation = m_relativeRotation;
        }
        
        return new Pose2d(fieldX, fieldY, Rotation2d.fromDegrees(fieldRotation));
    }

    @Override
    public void initialize() {
        // Convert alliance-relative coordinates to field coordinates
        m_targetPose = convertToFieldCoordinates();
        
        // Set PID setpoints
        m_xController.setSetpoint(m_targetPose.getX());
        m_yController.setSetpoint(m_targetPose.getY());
        m_rotController.setSetpoint(m_targetPose.getRotation().getDegrees());
        
        System.out.println("GoToPositionRelative: Navigating to " + m_alliance + " alliance position");
        System.out.println("  Relative coords: (" + m_relativeX + ", " + m_relativeY + ", " + m_relativeRotation + "°)");
        System.out.println("  Field coords: " + m_targetPose);
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
            System.out.println("GoToPositionRelative: Command interrupted");
        } else {
            System.out.println("GoToPositionRelative: Arrived at target position");
            System.out.println("  Final pose: " + m_drivetrain.getPose());
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when all controllers are at setpoint
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               m_rotController.atSetpoint();
    }
}
