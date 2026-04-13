package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Shooter;

/**
 * Runs the ball intake roller and the shooter feeder in reverse while held.
 *
 * Both subsystems are required, so this command will interrupt any active
 * ShootCommand or VortexSpeedShotCommand. Because those commands call
 * stopShooter() in their end() handler, isShooterActive is guaranteed false
 * before this command executes — meaning the isCharged() gate in
 * Shooter.updateMotors() cannot fire and the feeder runs in reverse cleanly.
 */
public class ReverseBallIntakeAndFeederCommand extends Command {
    private final BallIntake m_ballIntake;
    private final Shooter m_shooter;

    public ReverseBallIntakeAndFeederCommand(BallIntake ballIntake, Shooter shooter) {
        m_ballIntake = ballIntake;
        m_shooter = shooter;
        addRequirements(ballIntake, shooter);
    }

    @Override
    public void initialize() {
        m_shooter.stopShooter();
        m_shooter.stopFeeder();
        m_ballIntake.stopRun();
        m_ballIntake.reverseRun();
        m_shooter.reverseFeeder();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        m_ballIntake.stopReverseRun();
        m_shooter.stopReverseFeeder();
    }

    @Override
    public boolean isFinished() {
        return false; // Hold-to-run; ended by button release
    }
}
