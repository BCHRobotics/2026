package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * Simple autonomous command that drives forward a specified distance.
 * Example autonomous command for students to learn from.
 */
public class DriveForwardCommand extends Command {
    private final Drivetrain m_drivetrain;
    private final double m_targetDistance;
    private final double m_speed;
    private double m_startingX;

    /**
     * Creates a new DriveForwardCommand.
     *
     * @param drivetrain The drivetrain subsystem
     * @param distanceMeters How far to drive forward (in meters)
     * @param speed Drive speed (0.0 to 1.0)
     */
    public DriveForwardCommand(Drivetrain drivetrain, double distanceMeters, double speed) {
        m_drivetrain = drivetrain;
        m_targetDistance = distanceMeters;
        m_speed = Math.abs(speed); // Ensure positive

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Record starting position
        m_startingX = m_drivetrain.getPose().getX();
    }

    @Override
    public void execute() {
        // Drive forward at constant speed (robot-relative)
        m_drivetrain.drive(m_speed, 0, 0, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop when command ends
        m_drivetrain.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        // Check if we've traveled the target distance
        double currentX = m_drivetrain.getPose().getX();
        double distanceTraveled = Math.abs(currentX - m_startingX);
        return distanceTraveled >= m_targetDistance;
    }
}
