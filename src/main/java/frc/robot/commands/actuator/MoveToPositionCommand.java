package frc.robot.commands.actuator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Actuator;

/**
 * Command to move an actuator to a specific position using PID control.
 * This command finishes when the actuator reaches the target position.
 */
public class MoveToPositionCommand extends Command {
    private final Actuator m_actuator;
    private final double m_targetPosition;
    private static final double kPositionTolerance = 1.0; // rotations

    /**
     * Creates a new MoveToPositionCommand.
     *
     * @param actuator The actuator subsystem to control
     * @param targetPosition Target position in rotations
     */
    public MoveToPositionCommand(Actuator actuator, double targetPosition) {
        m_actuator = actuator;
        m_targetPosition = targetPosition;
        addRequirements(actuator);
    }

    @Override
    public void initialize() {
        // Set the target position - subsystem handles PID control
        m_actuator.setPosition(m_targetPosition);
    }

    @Override
    public void execute() {
        // PID runs in subsystem periodic() - nothing to do here
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Stop if interrupted
            m_actuator.stop();
        }
        // Otherwise leave at target position
    }

    @Override
    public boolean isFinished() {
        // Check if we're within tolerance of target
        double error = Math.abs(m_actuator.getPosition() - m_targetPosition);
        return error < kPositionTolerance;
    }
}
