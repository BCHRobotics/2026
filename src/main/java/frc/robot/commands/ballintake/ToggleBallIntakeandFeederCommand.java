package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Shooter;

/** Toggles the ball intake roller on or off. */
public class ToggleBallIntakeandFeederCommand extends Command {
  private final BallIntake m_ballIntake;
  private final Shooter m_shooter;

  public ToggleBallIntakeandFeederCommand(BallIntake ballIntake, Shooter shooter) {
    m_ballIntake = ballIntake;
    m_shooter = shooter;
    addRequirements(ballIntake, shooter);
  }

  @Override
  public void initialize() {
    m_ballIntake.toggleReverseRun();
    m_shooter.toggleReverseFeeder();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
