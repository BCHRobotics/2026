package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallIntakeConstants;

/**
 * Ball Intake subsystem with extend/retract and roller mechanisms.
 *
 * Hardware:
 *   - Extend/Retract: NEO Vortex, SPARK Flex, CAN ID 4
 *   - Run (roller):   NEO motor,  SPARK MAX,  CAN ID 6
 *
 * The calibrate() function slowly retracts the arm until a current spike
 * indicates contact with the hard stop, or a timeout expires.
 */
public class BallIntake extends SubsystemBase {
  private static final double JIGGLE_INTAKE_POSITION =
      BallIntakeConstants.kRetractedPosition
          + ((BallIntakeConstants.kExtendedPosition - BallIntakeConstants.kRetractedPosition) * 0.5);
  private static final double JIGGLE_INTERVAL_SECONDS = 0.5;

  // ── Calibration state machine ────────────────────────────────────────────
  private enum CalibrateState { IDLE, CALIBRATING, CALIBRATED, FAILED }

  // ── Motors ───────────────────────────────────────────────────────────────
  private final SparkFlex m_extendMotor;
  private final SparkMax  m_runMotor;
  private final RelativeEncoder m_extendEncoder;
  private final SparkClosedLoopController m_extendController;

  // ── Calibration tracking ─────────────────────────────────────────────────
  private CalibrateState m_calibrateState = CalibrateState.IDLE;
  private final Timer m_calibrateTimer = new Timer();
  private final Timer m_jiggleTimer = new Timer();
  private final LinearFilter m_calibrateCurrentFilter = LinearFilter.movingAverage(20);
  private boolean m_extendEnabled = false;
  private boolean m_runEnabled = false;
  private double m_targetExtendPosition = BallIntakeConstants.kRetractedPosition;
  private double m_filteredExtendCurrent = 0.0;
  private boolean m_jiggleIntakeActive = false;
  private boolean m_jiggleReturningToStart = false;
  private boolean m_jiggleStartedExtended = false;
  private double m_jiggleReturnPosition = BallIntakeConstants.kRetractedPosition;

  // ─────────────────────────────────────────────────────────────────────────

  public BallIntake() {
    m_extendMotor = new SparkFlex(BallIntakeConstants.kExtendMotorCanId, MotorType.kBrushless);
    m_runMotor    = new SparkMax(BallIntakeConstants.kRunMotorCanId,     MotorType.kBrushless);
    m_extendEncoder = m_extendMotor.getEncoder();
    m_extendController = m_extendMotor.getClosedLoopController();

    SparkFlexConfig extendConfig = new SparkFlexConfig();
    extendConfig
        .inverted(BallIntakeConstants.kExtendMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(BallIntakeConstants.kExtendCurrentLimit);
    extendConfig.encoder
      .positionConversionFactor(BallIntakeConstants.kExtendPositionConversionFactor)
      .velocityConversionFactor(BallIntakeConstants.kExtendVelocityConversionFactor);
    extendConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(
        BallIntakeConstants.kExtendPositionP,
        BallIntakeConstants.kExtendPositionI,
        BallIntakeConstants.kExtendPositionD)
      .outputRange(-1.0, 1.0);

    SparkMaxConfig runConfig = new SparkMaxConfig();
    runConfig
        .inverted(BallIntakeConstants.kRunMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(BallIntakeConstants.kRunCurrentLimit);

    m_extendMotor.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_runMotor.configure(runConfig,    ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_extendEncoder.setPosition(BallIntakeConstants.kRetractedPosition);
  }

  // ── Extend / Retract ─────────────────────────────────────────────────────

  /** Extends the intake arm at the configured speed. */
  public void extend() {
    m_extendEnabled = true;
    m_targetExtendPosition = BallIntakeConstants.kExtendedPosition;
    m_extendMotor.set(BallIntakeConstants.kExtendSpeed);
  }

  /** Retracts the intake arm at the configured speed. */
  public void retract() {
    m_extendEnabled = false;
    m_targetExtendPosition = BallIntakeConstants.kRetractedPosition;
    m_extendMotor.set(BallIntakeConstants.kRetractSpeed);
  }

  /** Moves the intake arm to the configured extended position. */
  public void moveToExtendedPosition() {
    if (!isCalibrated()) {
      return;
    }

    m_extendEnabled = true;
    m_targetExtendPosition = BallIntakeConstants.kExtendedPosition;
    setExtendPosition(m_targetExtendPosition);
  }

  /** Moves the intake arm back to the calibrated retracted position. */
  public void moveToRetractedPosition() {
    if (!isCalibrated()) {
      return;
    }

    m_extendEnabled = false;
    m_targetExtendPosition = BallIntakeConstants.kRetractedPosition;
    setExtendPosition(m_targetExtendPosition);
  }

  /** Toggles the intake arm between retracted and the configured extended position. */
  public void toggleExtendPosition() {
    if (m_extendEnabled) {
      moveToRetractedPosition();
      return;
    }

    moveToExtendedPosition();
  }

  /**
   * Moves the intake to the half-retracted position, waits 500 ms, then returns
   * it to the state it started from.
   */
  public void JiggleIntake() {
    if (!isCalibrated() || m_jiggleIntakeActive) {
      return;
    }

    m_jiggleIntakeActive = true;
    m_jiggleReturningToStart = false;
    m_jiggleStartedExtended = m_extendEnabled;
    m_jiggleReturnPosition = m_extendEnabled
        ? BallIntakeConstants.kExtendedPosition
        : BallIntakeConstants.kRetractedPosition;
    m_jiggleTimer.restart();

    m_targetExtendPosition = JIGGLE_INTAKE_POSITION;
    setExtendPosition(m_targetExtendPosition);
  }

  /** Stops the extend/retract motor. */
  public void stopExtend() {
    m_extendMotor.set(0.0);
  }

  /**
   * Manually drives the extend mechanism while enforcing the calibrated travel range.
     * Positive speed extends, negative speed retracts, and zero stops.
   */
    public void moveWhileHeld(double speed) {
      moveWhileHeld(speed, false);
    }

  /**
   * Manually drives the extend mechanism. When overrideCalibrationAndLimits is false,
   * movement requires calibration and is clamped to the calibrated travel range.
   */
    public void moveWhileHeld(double speed, boolean overrideCalibrationAndLimits) {
    // Normal operation refuses manual motion until homing has established a valid zero point.
    // The override path intentionally bypasses that safeguard for recovery and testing.
    if (!overrideCalibrationAndLimits && !isCalibrated()) {
      stopExtend();
      return;
    }

      double clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    double currentPosition = getExtendPosition();

      if (clampedSpeed > 0.0) {
      if (!overrideCalibrationAndLimits && currentPosition >= BallIntakeConstants.kExtendedPosition) {
        m_extendEnabled = true;
        m_targetExtendPosition = BallIntakeConstants.kExtendedPosition;
        stopExtend();
        return;
      }

      m_extendEnabled = true;
      m_targetExtendPosition = BallIntakeConstants.kExtendedPosition;
        m_extendMotor.set(BallIntakeConstants.kExtendSpeed * clampedSpeed);
      return;
    }

      if (clampedSpeed < 0.0) {
      if (!overrideCalibrationAndLimits && currentPosition <= BallIntakeConstants.kRetractedPosition) {
        m_extendEnabled = false;
        m_targetExtendPosition = BallIntakeConstants.kRetractedPosition;
        stopExtend();
        return;
      }

      m_extendEnabled = false;
      m_targetExtendPosition = BallIntakeConstants.kRetractedPosition;
        m_extendMotor.set(Math.abs(BallIntakeConstants.kRetractSpeed) * clampedSpeed);
      return;
    }

    stopExtend();
  }

  // ── Run (roller) ─────────────────────────────────────────────────────────

  /** Runs the roller at the configured intake speed. */
  public void run() {
    m_runEnabled = true;
    m_runMotor.set(BallIntakeConstants.kRunSpeed);
  }

  /** Runs the roller in reverse to eject. */
  public void eject() {
    m_runEnabled = true;
    m_runMotor.set(BallIntakeConstants.kEjectSpeed);
  }

  /** Stops the run motor. */
  public void stopRun() {
    m_runEnabled = false;
    m_runMotor.set(0.0);
  }

  /** Toggles the intake roller on and off. */
  public void toggleRun() {
    if (m_runEnabled) {
      stopRun();
      return;
    }

    run();
  }

  /** Stops both motors. */
  public void stop() {
    stopExtend();
    stopRun();
  }

  // ── Calibration ──────────────────────────────────────────────────────────

  /**
   * Begins calibration: slowly retracts the arm until a current increase is
   * detected on the extend motor (indicating the hard stop) or the timeout
   * elapses. Monitor completion with {@link #isCalibrating()},
   * {@link #isCalibrated()}, and {@link #isCalibrationFailed()}.
   */
  public void calibrate() {
    m_calibrateState = CalibrateState.CALIBRATING;
    m_extendEnabled = false;
    m_targetExtendPosition = BallIntakeConstants.kRetractedPosition;
    m_calibrateCurrentFilter.reset();
    m_filteredExtendCurrent = 0.0;
    m_calibrateTimer.restart();
    m_extendMotor.set(BallIntakeConstants.kCalibrateSpeed);
  }

  /** Resets the calibration state and starts a fresh homing cycle. */
  public void restartCalibration() {
    m_calibrateState = CalibrateState.IDLE;
    calibrate();
  }

  /** @return true while calibration is actively running. */
  public boolean isCalibrating() {
    return m_calibrateState == CalibrateState.CALIBRATING;
  }

  /** @return true if calibration completed successfully (current spike found). */
  public boolean isCalibrated() {
    return m_calibrateState == CalibrateState.CALIBRATED;
  }

  /** @return true if the intake has a valid calibration and is commanded extended. */
  public boolean isExtendEnabled() {
    return m_extendEnabled;
  }

  /** @return true when the roller toggle is currently on. */
  public boolean isRunEnabled() {
    return m_runEnabled;
  }

  /** @return current extend position in configured encoder units. */
  public double getExtendPosition() {
    return m_extendEncoder.getPosition();
  }

  /** @return commanded extend target in configured encoder units. */
  public double getTargetExtendPosition() {
    return m_targetExtendPosition;
  }

  /** @return true when the extend mechanism is within tolerance of its target. */
  public boolean isAtTargetPosition() {
    return Math.abs(getExtendPosition() - m_targetExtendPosition)
        <= BallIntakeConstants.kExtendPositionTolerance;
  }

  private void setExtendPosition(double targetPosition) {
    m_extendController.setSetpoint(
        targetPosition,
        ControlType.kPosition,
        BallIntakeConstants.kExtendClosedLoopSlot);
  }

  /** @return true if calibration ended due to timeout without detecting a stop. */
  public boolean isCalibrationFailed() {
    return m_calibrateState == CalibrateState.FAILED;
  }

  // ── Telemetry helpers ────────────────────────────────────────────────────

  /** @return Output current of the extend/retract motor in amps. */
  public double getExtendCurrent() {
    return m_extendMotor.getOutputCurrent();
  }

  /** @return Output current of the run motor in amps. */
  public double getRunCurrent() {
    return m_runMotor.getOutputCurrent();
  }

  // ── Periodic ─────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    double extendCurrent = m_extendMotor.getOutputCurrent();
    boolean currentSpike = false;

    // Calibration state machine
    if (m_calibrateState == CalibrateState.CALIBRATING) {
      // 5 samples at the 20 ms robot loop gives a 100 ms moving average.
      m_filteredExtendCurrent = m_calibrateCurrentFilter.calculate(extendCurrent);
      currentSpike  = m_filteredExtendCurrent >= BallIntakeConstants.kCalibrateCurrentThreshold;
      boolean timedOut      = m_calibrateTimer.hasElapsed(BallIntakeConstants.kCalibrateTimeoutSeconds);

      if (currentSpike) {
        m_extendMotor.set(0.0);
        m_extendEncoder.setPosition(BallIntakeConstants.kRetractedPosition);
        m_calibrateState = CalibrateState.CALIBRATED;
        setExtendPosition(BallIntakeConstants.kRetractedPosition);
      } else if (timedOut) {
        m_extendMotor.set(0.0);
        m_calibrateState = CalibrateState.FAILED;
      }
    } else {
      m_filteredExtendCurrent = extendCurrent;
    }

    if (m_jiggleIntakeActive) {
      if (!m_jiggleReturningToStart && m_jiggleTimer.hasElapsed(JIGGLE_INTERVAL_SECONDS)) {
        m_jiggleReturningToStart = true;
        m_extendEnabled = m_jiggleStartedExtended;
        m_targetExtendPosition = m_jiggleReturnPosition;
        setExtendPosition(m_targetExtendPosition);
        m_jiggleTimer.restart();
      } else if (m_jiggleReturningToStart && m_jiggleTimer.hasElapsed(JIGGLE_INTERVAL_SECONDS)) {
        m_jiggleIntakeActive = false;
        m_jiggleReturningToStart = false;
        m_extendEnabled = m_jiggleStartedExtended;
        m_jiggleTimer.stop();
      }
    }

    SmartDashboard.putString("BallIntake/CalibrateState", m_calibrateState.toString());
    SmartDashboard.putNumber("BallIntake/ExtendCurrent",  extendCurrent);
    SmartDashboard.putNumber("BallIntake/FilteredExtendCurrent", m_filteredExtendCurrent);
        SmartDashboard.putBoolean("BallIntake/current spike", currentSpike );

    SmartDashboard.putNumber("BallIntake/ExtendPosition", getExtendPosition());
    SmartDashboard.putNumber("BallIntake/TargetExtendPosition", m_targetExtendPosition);
    SmartDashboard.putBoolean("BallIntake/ExtendEnabled", m_extendEnabled);
    // SmartDashboard.putBoolean("BallIntake/JiggleActive", m_jiggleIntakeActive);
    // SmartDashboard.putBoolean("BallIntake/RunEnabled", m_runEnabled);
    SmartDashboard.putBoolean("BallIntake/AtTarget", isAtTargetPosition());
    SmartDashboard.putNumber("BallIntake/RunCurrent",     m_runMotor.getOutputCurrent());
  }
}