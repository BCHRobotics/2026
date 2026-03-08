package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;

/** Starts intake calibration and finishes when calibration succeeds or fails. */
public class CalibrateBallIntakeCommand extends Command {
  private final BallIntake m_ballIntake;

  public CalibrateBallIntakeCommand(BallIntake ballIntake) {
    m_ballIntake = ballIntake;
    addRequirements(ballIntake);
  }

  @Override
  public void initialize() {
    m_ballIntake.restartCalibration();
  }

  @Override
  public boolean isFinished() {
    return m_ballIntake.isCalibrated() || m_ballIntake.isCalibrationFailed();
  }
}