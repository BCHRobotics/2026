package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
  private static final double kRobotLoopPeriodSeconds = 0.02;
  private static final double kMotionTimeoutSeconds = 5.0;
  private static final int kCurrentAverageSamples = Math.max(
      1,
      (int) Math.round(ClimbConstants.kCurrentAverageWindowSeconds / kRobotLoopPeriodSeconds));

  private enum MotionState {
    STOPPED,
    EXTENDING,
    RETRACTING
  }

  private final SparkMax climbMotor;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final DigitalInput proximitySwitch;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(kCurrentAverageSamples);
  private final Timer motionTimer = new Timer();
  private double filteredCurrent = 0.0;
  private MotionState motionState = MotionState.STOPPED;

  public Climber() {
    climbMotor = new SparkMax(ClimbConstants.kMotorCanId, MotorType.kBrushless);
    // REV exposes the physical limit inputs through the motor controller, so we can read them
    // without wiring a separate DIO input on the roboRIO.
    forwardLimitSwitch = climbMotor.getForwardLimitSwitch();
    reverseLimitSwitch = climbMotor.getReverseLimitSwitch();
    proximitySwitch = new DigitalInput(ClimbConstants.kProximitySwitchChannel);

    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig
        .inverted(ClimbConstants.kMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimbConstants.kMotorCurrentLimit);

    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void extendClimber() {
    setMotionState(MotionState.EXTENDING);

    if (isExtendLimitReached()) {
      stop();
      return;
    }

    climbMotor.set(ClimbConstants.kExtendSpeed);
  }

  public void retractClimber() {
    setMotionState(MotionState.RETRACTING);

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
    setMotionState(MotionState.STOPPED);
    climbMotor.set(0.0);
  }

  private void setMotionState(MotionState newState) {
    if (motionState == newState) {
      return;
    }

    motionState = newState;

    if (newState == MotionState.STOPPED) {
      motionTimer.stop();
      motionTimer.reset();
    } else {
      motionTimer.restart();
    }
  }

  private boolean hasMotionTimedOut(MotionState expectedState) {
    return motionState == expectedState && motionTimer.hasElapsed(kMotionTimeoutSeconds);
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

  public boolean isClimbPlateDetected() {
    boolean rawValue = proximitySwitch.get();
    return ClimbConstants.kProximitySwitchActiveLow ? !rawValue : rawValue;
  }

  public boolean isExtendLimitReached() {
    return isForwardLimitSwitchPressed()
        || isCurrentSpikeDetected()
        || hasMotionTimedOut(MotionState.EXTENDING);
  }

  public boolean isRetractLimitReached() {
    return isReverseLimitSwitchPressed()
        || isCurrentSpikeDetected()
        || hasMotionTimedOut(MotionState.RETRACTING);
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
    SmartDashboard.putBoolean("Climber/PlateDetected", isClimbPlateDetected());
    SmartDashboard.putBoolean("Climber/ExtendLimitReached", isExtendLimitReached());
    SmartDashboard.putBoolean("Climber/RetractLimitReached", isRetractLimitReached());
    SmartDashboard.putBoolean("Climber/MotionTimedOut", hasMotionTimedOut(MotionState.EXTENDING)
        || hasMotionTimedOut(MotionState.RETRACTING));
    SmartDashboard.putString("Climber/MotionState", motionState.name());
    SmartDashboard.putNumber("Climber/Current", climbMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/FilteredCurrent", filteredCurrent);
    SmartDashboard.putNumber("Climber/AppliedOutput", climbMotor.getAppliedOutput());
  }
}