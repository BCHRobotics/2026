package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;

/** Toggles the ball intake between its calibrated retracted and extended positions. */
public class ToggleBallIntakeExtendCommand extends Command {
  private final BallIntake m_ballIntake;

  public ToggleBallIntakeExtendCommand(BallIntake ballIntake) {
    m_ballIntake = ballIntake;
    addRequirements(ballIntake);
  }

  @Override
  public void initialize() {
    m_ballIntake.toggleExtendPosition();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}