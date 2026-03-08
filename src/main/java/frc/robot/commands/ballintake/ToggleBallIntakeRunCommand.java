package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;

/** Toggles the ball intake roller on or off. */
public class ToggleBallIntakeRunCommand extends Command {
  private final BallIntake m_ballIntake;

  public ToggleBallIntakeRunCommand(BallIntake ballIntake) {
    m_ballIntake = ballIntake;
    addRequirements(ballIntake);
  }

  @Override
  public void initialize() {
    m_ballIntake.toggleRun();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}