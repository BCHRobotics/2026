package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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
import org.littletonrobotics.junction.Logger;
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

  private enum CalibrateState { IDLE, CALIBRATING, CALIBRATED, FAILED }

  private final SparkMax climbMotor;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final DigitalInput proximitySwitch;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(kCurrentAverageSamples);
  private final Timer motionTimer = new Timer();
  private double filteredCurrent = 0.0;
  private MotionState motionState = MotionState.STOPPED;
  private CalibrateState calibrateState = CalibrateState.IDLE;

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

    // Acquire the built-in encoder and zero it now so 0 = retracted.
    encoder = climbMotor.getEncoder();
    encoder.setPosition(0.0);
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

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public boolean isExtendLimitReached() {
    return isForwardLimitSwitchPressed()
        || isCurrentSpikeDetected()
        || hasMotionTimedOut(MotionState.EXTENDING)
        || getEncoderPosition() >= ClimbConstants.kEncoderExtendedPosition;
  }

  public boolean isRetractLimitReached() {
    return isReverseLimitSwitchPressed()
        || isCurrentSpikeDetected()
        || hasMotionTimedOut(MotionState.RETRACTING);
  }

  public boolean isLimitSwitchPressed() {
    return isForwardLimitSwitchPressed() || isReverseLimitSwitchPressed();
  }

  /**
   * Starts a calibration run: retracts the climber until a current spike or reverse
   * limit switch is detected, then zeros the encoder. Driven from periodic().
   */
  public void startCalibration() {
    calibrateState = CalibrateState.CALIBRATING;
    setMotionState(MotionState.RETRACTING); // starts the motion timeout timer
  }

  public boolean isCalibrated() {
    return calibrateState == CalibrateState.CALIBRATED;
  }

  public boolean isCalibrationFailed() {
    return calibrateState == CalibrateState.FAILED;
  }

  @Override
  public void periodic() {
    // Run calibration state machine before updating filtered current so the spike
    // check below uses the value from the previous loop (consistent with normal use).
    if (calibrateState == CalibrateState.CALIBRATING) {
      if (hasMotionTimedOut(MotionState.RETRACTING)) {
        climbMotor.set(0.0);
        setMotionState(MotionState.STOPPED);
        calibrateState = CalibrateState.FAILED;
      } else if (isReverseLimitSwitchPressed() || isCurrentSpikeDetected()) {
        climbMotor.set(0.0);
        encoder.setPosition(0.0);
        setMotionState(MotionState.STOPPED);
        calibrateState = CalibrateState.CALIBRATED;
      } else {
        climbMotor.set(ClimbConstants.kRetractSpeed);
      }
    }

    filteredCurrent = currentFilter.calculate(climbMotor.getOutputCurrent());

    // Publish the important climber signals so students can debug the mechanism from the dashboard.
    SmartDashboard.putString("Climber/CalibrateState", calibrateState.name());
    Logger.recordOutput("Climber/CalibrateState", calibrateState.name());

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

    Logger.recordOutput("Climber/ForwardLimitPressed", isForwardLimitSwitchPressed());
    Logger.recordOutput("Climber/ReverseLimitPressed", isReverseLimitSwitchPressed());
    Logger.recordOutput("Climber/LimitPressed", isLimitSwitchPressed());
    Logger.recordOutput("Climber/CurrentSpikeDetected", isCurrentSpikeDetected());
    Logger.recordOutput("Climber/PlateDetected", isClimbPlateDetected());
    Logger.recordOutput("Climber/ExtendLimitReached", isExtendLimitReached());
    Logger.recordOutput("Climber/RetractLimitReached", isRetractLimitReached());
    Logger.recordOutput("Climber/MotionTimedOut", hasMotionTimedOut(MotionState.EXTENDING)
        || hasMotionTimedOut(MotionState.RETRACTING));
    Logger.recordOutput("Climber/MotionState", motionState.name());
    Logger.recordOutput("Climber/Current", climbMotor.getOutputCurrent());
    Logger.recordOutput("Climber/FilteredCurrent", filteredCurrent);
    Logger.recordOutput("Climber/AppliedOutput", climbMotor.getAppliedOutput());
    Logger.recordOutput("Climber/EncoderPosition", getEncoderPosition());
    SmartDashboard.putNumber("Climber/EncoderPosition", getEncoderPosition());
  }
}