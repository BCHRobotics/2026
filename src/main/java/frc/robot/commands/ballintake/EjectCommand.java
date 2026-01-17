package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;

/**
 * Command to eject balls using the ball intake subsystem.
 * 
 * This command runs the intake motor in reverse at the configured eject speed
 * until interrupted or manually stopped.
 */
public class EjectCommand extends Command {
  private final BallIntake m_intake;

  /**
   * Creates a new EjectCommand.
   *
   * @param intake The ball intake subsystem to control
   */
  public EjectCommand(BallIntake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.eject();
  }

  @Override
  public void execute() {
    // Eject speed is maintained by the subsystem
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the intake when command ends
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted
    return false;
  }
}