package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
  private static final double kRobotLoopPeriodSeconds = 0.02;
  private static final int kCurrentAverageSamples = Math.max(
      1,
      (int) Math.round(ClimbConstants.kCurrentAverageWindowSeconds / kRobotLoopPeriodSeconds));

  private final SparkMax climbMotor;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(kCurrentAverageSamples);
  private double filteredCurrent = 0.0;

  public Climber() {
    climbMotor = new SparkMax(ClimbConstants.kMotorCanId, MotorType.kBrushless);
    // REV exposes the physical limit inputs through the motor controller, so we can read them
    // without wiring a separate DIO input on the roboRIO.
    forwardLimitSwitch = climbMotor.getForwardLimitSwitch();
    reverseLimitSwitch = climbMotor.getReverseLimitSwitch();

    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig
        .inverted(ClimbConstants.kMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimbConstants.kMotorCurrentLimit);

    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void extendClimber() {
    if (isExtendLimitReached()) {
      stop();
      return;
    }

    climbMotor.set(ClimbConstants.kExtendSpeed);
  }

  public void retractClimber() {
    if (isRetractLimitReached()) {
      stop();
      return;
    }

    climbMotor.set(ClimbConstants.kRetractSpeed);
  }

  public void runClimber() {
    extendClimber();
  }

  public void stop() {
    climbMotor.set(0.0);
  }

  public boolean isForwardLimitSwitchPressed() {
    return forwardLimitSwitch.isPressed();
  }

  public boolean isReverseLimitSwitchPressed() {
    return reverseLimitSwitch.isPressed();
  }

  public boolean isCurrentSpikeDetected() {
    return Math.abs(climbMotor.getAppliedOutput()) > 1e-3
        && filteredCurrent >= ClimbConstants.kCurrentSpikeThresholdAmps;
  }

  public boolean isExtendLimitReached() {
    return isForwardLimitSwitchPressed() || isCurrentSpikeDetected();
  }

  public boolean isRetractLimitReached() {
    return isReverseLimitSwitchPressed() || isCurrentSpikeDetected();
  }

  public boolean isLimitSwitchPressed() {
    return isForwardLimitSwitchPressed() || isReverseLimitSwitchPressed();
  }

  @Override
  public void periodic() {
    filteredCurrent = currentFilter.calculate(climbMotor.getOutputCurrent());

    // Publish the important climber signals so students can debug the mechanism from the dashboard.
    SmartDashboard.putBoolean("Climber/ForwardLimitPressed", isForwardLimitSwitchPressed());
    SmartDashboard.putBoolean("Climber/ReverseLimitPressed", isReverseLimitSwitchPressed());
    SmartDashboard.putBoolean("Climber/LimitPressed", isLimitSwitchPressed());
    SmartDashboard.putBoolean("Climber/CurrentSpikeDetected", isCurrentSpikeDetected());
    SmartDashboard.putBoolean("Climber/ExtendLimitReached", isExtendLimitReached());
    SmartDashboard.putBoolean("Climber/RetractLimitReached", isRetractLimitReached());
    SmartDashboard.putNumber("Climber/Current", climbMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/FilteredCurrent", filteredCurrent);
    SmartDashboard.putNumber("Climber/AppliedOutput", climbMotor.getAppliedOutput());
  }
}