package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Command to run the feeder and shoot game pieces.
 * 
 * This command:
 * 1. Waits for flywheel to reach minimum threshold RPM
 * 2. Runs the feeder to feed game pieces into the flywheel
 * 3. Continues running while the button is held
 * 4. Stops the feeder when released (flywheel keeps spinning)
 * 
 * Safety: Feeder will not run if flywheel is below threshold RPM.
 * 
 * Usage: Bind to a button with whileTrue() to feed while held.
 * Example: driverController.R1().whileTrue(new ShootCommand(m_shooter))
 */
public class ShootCommand extends Command {
    private final Shooter m_shooter;

    /**
     * Creates a new ShootCommand.
     *
     * @param shooter The shooter subsystem to control
     */
    public ShootCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    /** Resets state and starts the flywheel spinning */
    @Override
    public void initialize() {
        m_shooter.startShooter();
        m_shooter.stopFeeder();
    }

    /** Feeds game pieces when flywheel reaches threshold RPM, otherwise waits */
    @Override
    public void execute() {
        // Run feeder if flywheel is ready
        if (m_shooter.isCharged()) {
            m_shooter.startFeeder();
        } else {
            // Flywheel not ready yet - wait for it to spin up
            m_shooter.stopFeeder();
        }
    }

    /** Stops both feeder and flywheel when command ends (button released) */
    @Override
    public void end(boolean interrupted) {
        // Stop feeder when command ends (button released)
        // Flywheel continues running for quick follow-up shots
        m_shooter.stopShooter();
        m_shooter.stopFeeder();
    }

    /** Command runs continuously while button is held */
    @Override
    public boolean isFinished() {
        // Never finishes - runs until button is released
        return false;
    }
}
