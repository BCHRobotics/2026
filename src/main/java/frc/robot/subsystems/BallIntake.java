package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallIntakeConstants;

/**
 * Ball Intake subsystem with extend/retract and roller mechanisms.
 *
 * Hardware:
 *   - Extend/Retract: NEO motor, SPARK MAX, CAN ID 4
 *   - Run (roller):   NEO motor, SPARK MAX, CAN ID 6
 *
 * The calibrate() function slowly retracts the arm until a current spike
 * indicates contact with the hard stop, or a timeout expires.
 */
public class BallIntake extends SubsystemBase {

  // ── Calibration state machine ────────────────────────────────────────────
  private enum CalibrateState { IDLE, CALIBRATING, CALIBRATED, FAILED }

  // ── Motors ───────────────────────────────────────────────────────────────
  private final SparkMax m_extendMotor;
  private final SparkMax m_runMotor;

  // ── Calibration tracking ─────────────────────────────────────────────────
  private CalibrateState m_calibrateState = CalibrateState.IDLE;
  private final Timer m_calibrateTimer = new Timer();

  // ─────────────────────────────────────────────────────────────────────────

  public BallIntake() {
    m_extendMotor = new SparkMax(BallIntakeConstants.kExtendMotorCanId, MotorType.kBrushless);
    m_runMotor    = new SparkMax(BallIntakeConstants.kRunMotorCanId,    MotorType.kBrushless);

    SparkMaxConfig extendConfig = new SparkMaxConfig();
    extendConfig
        .inverted(BallIntakeConstants.kExtendMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(BallIntakeConstants.kExtendCurrentLimit);

    SparkMaxConfig runConfig = new SparkMaxConfig();
    runConfig
        .inverted(BallIntakeConstants.kRunMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(BallIntakeConstants.kRunCurrentLimit);

    m_extendMotor.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_runMotor.configure(runConfig,    ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ── Extend / Retract ─────────────────────────────────────────────────────

  /** Extends the intake arm at the configured speed. */
  public void extend() {
    m_extendMotor.set(BallIntakeConstants.kExtendSpeed);
  }

  /** Retracts the intake arm at the configured speed. */
  public void retract() {
    m_extendMotor.set(BallIntakeConstants.kRetractSpeed);
  }

  /** Stops the extend/retract motor. */
  public void stopExtend() {
    m_extendMotor.set(0.0);
  }

  // ── Run (roller) ─────────────────────────────────────────────────────────

  /** Runs the roller at the configured intake speed. */
  public void run() {
    m_runMotor.set(BallIntakeConstants.kRunSpeed);
  }

  /** Runs the roller in reverse to eject. */
  public void eject() {
    m_runMotor.set(BallIntakeConstants.kEjectSpeed);
  }

  /** Stops the run motor. */
  public void stopRun() {
    m_runMotor.set(0.0);
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
    m_calibrateTimer.restart();
    m_extendMotor.set(BallIntakeConstants.kCalibrateSpeed);
  }

  /** @return true while calibration is actively running. */
  public boolean isCalibrating() {
    return m_calibrateState == CalibrateState.CALIBRATING;
  }

  /** @return true if calibration completed successfully (current spike found). */
  public boolean isCalibrated() {
    return m_calibrateState == CalibrateState.CALIBRATED;
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
    // Calibration state machine
    if (m_calibrateState == CalibrateState.CALIBRATING) {
      boolean currentSpike  = m_extendMotor.getOutputCurrent() >= BallIntakeConstants.kCalibrateCurrentThreshold;
      boolean timedOut      = m_calibrateTimer.hasElapsed(BallIntakeConstants.kCalibrateTimeoutSeconds);

      if (currentSpike) {
        m_extendMotor.set(0.0);
        m_calibrateState = CalibrateState.CALIBRATED;
      } else if (timedOut) {
        m_extendMotor.set(0.0);
        m_calibrateState = CalibrateState.FAILED;
      }
    }

    SmartDashboard.putString("BallIntake/CalibrateState", m_calibrateState.toString());
    SmartDashboard.putNumber("BallIntake/ExtendCurrent",  m_extendMotor.getOutputCurrent());
    SmartDashboard.putNumber("BallIntake/RunCurrent",     m_runMotor.getOutputCurrent());
  }
}