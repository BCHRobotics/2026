package frc.robot.commands.ballintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntake;

import java.util.function.BooleanSupplier;

/** Moves the intake extension while held, bounded to the calibrated travel range. */
public class HoldBallIntakeExtendCommand extends Command {
  private final BallIntake m_ballIntake;
  private final double m_speed;
  private final BooleanSupplier m_overrideCalibrationAndLimitsSupplier;

  public HoldBallIntakeExtendCommand(BallIntake ballIntake, double speed) {
    this(ballIntake, speed, () -> false);
  }

  public HoldBallIntakeExtendCommand(
      BallIntake ballIntake,
      double speed,
      BooleanSupplier overrideCalibrationAndLimitsSupplier) {
    m_ballIntake = ballIntake;
    m_speed = speed;
    m_overrideCalibrationAndLimitsSupplier = overrideCalibrationAndLimitsSupplier;
    addRequirements(ballIntake);
  }

  @Override
  public void execute() {
    m_ballIntake.moveWhileHeld(m_speed, m_overrideCalibrationAndLimitsSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    m_ballIntake.stopExtend();
  }
}