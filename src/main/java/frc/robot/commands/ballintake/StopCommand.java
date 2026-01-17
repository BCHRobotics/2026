package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;

/**
 * Command to stop the ball intake subsystem.
 * 
 * This command immediately stops the intake motor.
 */
public class StopCommand extends Command {
  private final BallIntake m_intake;

  /**
   * Creates a new StopCommand.
   *
   * @param intake The ball intake subsystem to control
   */
  public StopCommand(BallIntake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.stop();
  }

  @Override
  public void execute() {
    // Motor is already stopped
  }

  @Override
  public void end(boolean interrupted) {
    // Command is instantaneous
  }

  @Override
  public boolean isFinished() {
    // This command completes immediately
    return true;
  }
}