package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/** Retracts the climber until a current spike or reverse limit switch is hit, then zeros the encoder. */
public class CalibrateClimberCommand extends Command {
  private final Climber climber;

  public CalibrateClimberCommand(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.startCalibration();
  }

  @Override
  public boolean isFinished() {
    return climber.isCalibrated() || climber.isCalibrationFailed();
  }
}
