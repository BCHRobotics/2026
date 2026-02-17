package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallIntakeConstants;

/**
 * Ball Intake subsystem for collecting and controlling game pieces.
 * 
 * This subsystem controls a variable speed motor for:
 * - Intaking balls at configurable speed
 * - Ejecting balls when needed
 * - Holding balls with low speed
 * - Stopping intake completely
 * 
 * Hardware: Single NEO motor with SPARK MAX controller
 * 
 * REVLib 2025+ Configuration Pattern:
 * - Uses SparkMaxConfig for all configuration
 * - Applies configuration with configure() method
 * - Uses ResetMode and PersistMode for reliability
 */
public class BallIntake extends SubsystemBase {
  /** SPARK MAX motor controller for the intake */
  private final SparkMax m_motor;

  /** Current intake speed (-1.0 to 1.0) */
  private double m_currentSpeed = 0.0;

  /**
   * Creates a new BallIntake subsystem.
   * 
   * Configures the motor using REVLib 2025+ best practices:
   * - Smart current limiting to prevent motor damage
   * - Coast mode for smooth operation
   * - Motor inversion based on constants
   * - Persistent configuration across power cycles
   */
  public BallIntake() {
    // Initialize motor
    m_motor = new SparkMax(BallIntakeConstants.kMotorCanId, MotorType.kBrushless);
    
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    
    /*
     * Configure motor parameters using the new fluent API.
     * This replaces the old direct method calls on the motor controller.
     */
    motorConfig
        .inverted(BallIntakeConstants.kMotorInverted)
        .idleMode(IdleMode.kCoast)  // Coast mode for smooth intake operation
        .smartCurrentLimit(BallIntakeConstants.kCurrentLimit);
    
    /*
     * Apply the configuration to the SPARK MAX.
     *
     * ResetMode: Resets the SPARK MAX to a known state, useful if replaced.
     * PersistMode: Saves configuration to flash memory for power cycles.
     */
    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets the intake motor speed.
   * 
   * @param speed Motor speed from -1.0 (full reverse) to 1.0 (full forward).
   *              Values within deadband are treated as zero.
   */
  public void setSpeed(double speed) {
    // Apply deadband to prevent motor creep
    if (Math.abs(speed) < BallIntakeConstants.kSpeedDeadband) {
      speed = 0.0;
    }
    
    m_currentSpeed = speed;
    m_motor.set(speed);
  }

  /**
   * Starts intaking balls at the configured intake speed.
   */
  public void intake() {
    setSpeed(BallIntakeConstants.kIntakeSpeed);
  }

  /**
   * Ejects balls at the configured eject speed.
   */
  public void eject() {
    setSpeed(BallIntakeConstants.kEjectSpeed);
  }

  /**
   * Holds balls with low speed to prevent them from falling out.
   */
  public void hold() {
    setSpeed(BallIntakeConstants.kHoldSpeed);
  }

  /**
   * Stops the intake motor completely.
   */
  public void stop() {
    setSpeed(0.0);
  }

  /**
   * Gets the current motor speed.
   * 
   * @return Current speed setting (-1.0 to 1.0)
   */
  public double getSpeed() {
    return m_currentSpeed;
  }

  /**
   * Gets the motor output current.
   * 
   * @return Current draw in amps
   */
  public double getCurrent() {
    return m_motor.getOutputCurrent();
  }

  /**
   * Checks if the intake is currently running.
   * 
   * @return true if motor speed is above deadband threshold
   */
  public boolean isRunning() {
    return Math.abs(m_currentSpeed) >= BallIntakeConstants.kSpeedDeadband;
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with intake status
    SmartDashboard.putNumber("Ball Intake/Speed", m_currentSpeed);
    SmartDashboard.putNumber("Ball Intake/Current", getCurrent());
    SmartDashboard.putBoolean("Ball Intake/Running", isRunning());
  }
}