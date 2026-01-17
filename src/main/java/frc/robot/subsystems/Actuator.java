/*
 * Actuator.java
 * 
 * PID-Controlled Mechanical Actuator Subsystem
 * 
 * This subsystem controls a linear or rotary actuator using closed-loop PID control.
 * Designed for mechanisms like elevators, arms, intakes, or any position-controlled system.
 * 
 * Key Features:
 * - PID position control with integrated motor controller
 * - Soft limits for safety (prevents over-extension)
 * - Current limiting to protect mechanism
 * - Manual and automatic control modes
 * - Position and velocity telemetry
 * 
 * Hardware:
 * - REV NEO motor with SPARK MAX controller
 * - Built-in NEO encoder for position feedback
 * 
 * Control Modes:
 * - Manual: Direct voltage control via joystick (with limits)
 * - Automatic: PID position control to setpoints
 * 
 * Safety Features:
 * - Forward/reverse soft limits prevent mechanical damage
 * - Current limiting protects motor and mechanism
 * - Position clamping ensures setpoints stay in valid range
 * 
 * @see ActuatorConstants For all configuration parameters
 */

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorConstants;

public class Actuator extends SubsystemBase {
    
    /**
     * SPARK MAX motor controller for the actuator.
     * 
     * Uses NEO brushless motor for efficient, high-torque actuation.
     * Configured with PID control, current limits, and soft limits.
     */
    private final SparkMax motor;
    
    /**
     * Built-in encoder from NEO motor.
     * 
     * Tracks position (rotations) and velocity (RPM).
     * Zero position should be set at mechanism's home/retracted position.
     */
    private final RelativeEncoder encoder;
    
    /**
     * PID controller built into SPARK MAX.
     * 
     * Runs on-board for faster loop times and reduced CAN traffic.
     * Configured with PID constants from ActuatorConstants.
     */
    private final SparkClosedLoopController pidController;
    
    /**
     * Current setpoint position in rotations.
     * 
     * Tracked separately to avoid reading back from SPARK MAX every cycle.
     * Updated whenever a new position is commanded.
     */
    private double currentSetpoint = 0.0;
    
    /**
     * Creates a new Actuator subsystem.
     * 
     * Initializes motor controller with:
     * - PID configuration for position control
     * - Current limits to protect motor and mechanism
     * - Soft limits to prevent over-travel
     * - Brake mode for holding position
     * - Encoder configuration and zeroing
     */
    public Actuator() {
        // Initialize SPARK MAX motor controller
        motor = new SparkMax(ActuatorConstants.kMotorCanId, MotorType.kBrushless);
        
        // Create configuration object
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Configure basic motor settings
        config.inverted(ActuatorConstants.kMotorInverted)
              .idleMode(IdleMode.kBrake); // Brake mode holds position when idle
        
        // Configure current limits
        config.smartCurrentLimit(ActuatorConstants.kCurrentLimit);
        
        // Configure soft limits (prevents mechanism damage)
        config.softLimit.forwardSoftLimit(ActuatorConstants.kForwardSoftLimit)
                        .forwardSoftLimitEnabled(true)
                        .reverseSoftLimit(ActuatorConstants.kReverseSoftLimit)
                        .reverseSoftLimitEnabled(true);
        
        // Configure PID controller (slot 0)
        config.closedLoop.pid(ActuatorConstants.kP, ActuatorConstants.kI, ActuatorConstants.kD)
                         .outputRange(ActuatorConstants.kMinOutput, ActuatorConstants.kMaxOutput)
                         .maxMotion.maxVelocity(ActuatorConstants.kMaxVelocity)
                                   .maxAcceleration(ActuatorConstants.kMaxAcceleration);
        
        // Configure encoder
        config.encoder.positionConversionFactor(ActuatorConstants.kPositionConversionFactor)
                      .velocityConversionFactor(ActuatorConstants.kVelocityConversionFactor);
        
        // Apply configuration to motor controller
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Get encoder and PID controller references
        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();

        // Reset encoder to zero (assumes mechanism starts at home position)
        // If not at home, call resetPosition() after moving to home
        encoder.setPosition(0);
        
        // Initialize setpoint to current position
        currentSetpoint = encoder.getPosition();
    }
    
    /**
     * Sets the target position for PID control.
     * 
     * The actuator will move to this position using closed-loop control.
     * Position is automatically clamped to soft limit range.
     * 
     * @param position Target position in rotations (or custom units if conversion factor set)
     */
    public void setPosition(double position) {
        // Clamp position to safe range
        double clampedPosition = Math.max(ActuatorConstants.kReverseSoftLimit,
                                         Math.min(ActuatorConstants.kForwardSoftLimit, position));
        
        // Update setpoint
        currentSetpoint = clampedPosition;
        
        // Command PID controller to target position
        pidController.setReference(clampedPosition, SparkMax.ControlType.kPosition);
    }
    
    /**
     * Moves actuator with manual voltage control.
     * 
     * Used for joystick control. Speed is clamped and soft limits are still enforced
     * by the SPARK MAX hardware.
     * 
     * @param speed Speed from -1.0 (full reverse) to 1.0 (full forward)
     */
    public void setManualSpeed(double speed) {
        // Clamp speed to valid range
        speed = Math.max(-1.0, Math.min(1.0, speed));
        
        // Apply voltage directly (bypasses PID)
        motor.set(speed);
        
        // Update setpoint to current position (for smooth transition back to auto)
        currentSetpoint = encoder.getPosition();
    }
    
    /**
     * Stops the actuator motor.
     * 
     * In brake mode, the motor will hold its current position.
     */
    public void stop() {
        motor.stopMotor();
        currentSetpoint = encoder.getPosition();
    }
    
    /**
     * Gets the current position of the actuator.
     * 
     * @return Current position in rotations (or custom units if conversion factor set)
     */
    public double getPosition() {
        return encoder.getPosition();
    }
    
    /**
     * Gets the current velocity of the actuator.
     * 
     * @return Current velocity in rotations per second (or custom units)
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }
    
    /**
     * Gets the current setpoint position.
     * 
     * @return Target position that PID is driving toward
     */
    public double getSetpoint() {
        return currentSetpoint;
    }
    
    /**
     * Checks if actuator is at the target position.
     * 
     * @return true if within tolerance of setpoint
     */
    public boolean atSetpoint() {
        return Math.abs(getPosition() - currentSetpoint) < ActuatorConstants.kPositionTolerance;
    }
    
    /**
     * Resets the encoder to zero at the current position.
     * 
     * Call this when the mechanism is at a known home position.
     * Useful for re-establishing position reference after power cycle.
     */
    public void resetPosition() {
        encoder.setPosition(0);
        currentSetpoint = 0;
    }
    
    /**
     * Gets the current motor output percentage.
     * 
     * @return Applied output from -1.0 to 1.0
     */
    public double getMotorOutput() {
        return motor.getAppliedOutput();
    }
    
    /**
     * Gets the motor current draw.
     * 
     * @return Current in amps
     */
    public double getCurrent() {
        return motor.getOutputCurrent();
    }
    
    @Override
    public void periodic() {
        // Update telemetry to dashboard
        SmartDashboard.putNumber("Actuator/Position", getPosition());
        SmartDashboard.putNumber("Actuator/Velocity", getVelocity());
        SmartDashboard.putNumber("Actuator/Setpoint", getSetpoint());
        SmartDashboard.putBoolean("Actuator/At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Actuator/Current (A)", getCurrent());
        SmartDashboard.putNumber("Actuator/Output %", getMotorOutput());
    }
}
