package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;

/**
 * Command to hold balls using the ball intake subsystem.
 * 
 * This command runs the intake motor at low speed to retain balls
 * without collecting additional ones.
 */
public class HoldCommand extends Command {
  private final BallIntake m_intake;

  /**
   * Creates a new HoldCommand.
   *
   * @param intake The ball intake subsystem to control
   */
  public HoldCommand(BallIntake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.hold();
  }

  @Override
  public void execute() {
    // Hold speed is maintained by the subsystem
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