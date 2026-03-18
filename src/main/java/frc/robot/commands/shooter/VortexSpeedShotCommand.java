package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Command to run the shooter at 95% of NEO Vortex free speed and auto-feed once ready.
 */
public class VortexSpeedShotCommand extends Command {
    private final Shooter shooter;

    public VortexSpeedShotCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.startVortexSpeedShot();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}