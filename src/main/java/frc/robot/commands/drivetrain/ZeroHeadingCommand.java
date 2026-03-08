package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/** Resets the drivetrain heading so the current direction becomes field forward. */
public class ZeroHeadingCommand extends Command {
  private final Drivetrain m_drivetrain;

  public ZeroHeadingCommand(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_drivetrain.zeroHeading();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}