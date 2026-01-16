/*
 * Actuator2.java
 * 
 * WPILib PID-Controlled Mechanical Actuator Subsystem
 * 
 * This subsystem controls a linear or rotary actuator using WPILib's PIDController
 * instead of the SPARK MAX's onboard PID. This approach runs the control loop on the
 * RoboRIO, providing more flexibility for custom control logic and easier debugging.
 * 
 * Key Features:
 * - WPILib PIDController for position control (runs on RoboRIO)
 * - Software-enforced position limits
 * - Current limiting to protect mechanism
 * - Manual and automatic control modes
 * - Position and velocity telemetry
 * 
 * Hardware:
 * - REV NEO motor with SPARK MAX controller (voltage control only)
 * - Built-in NEO encoder for position feedback
 * 
 * Control Modes:
 * - Manual: Direct voltage control via joystick (with limits)
 * - Automatic: PID position control to setpoints (RoboRIO-based)
 * 
 * Safety Features:
 * - Software position limits prevent mechanical damage
 * - Current limiting protects motor and mechanism
 * - Position clamping ensures setpoints stay in valid range
 * 
 * Advantages of RoboRIO PID vs. Onboard PID:
 * - Easier to tune and debug via SmartDashboard
 * - Can implement custom feedforward and control logic
 * - Better integration with other subsystems
 * - More detailed telemetry and logging
 * 
 * @see Actuator2Constants For all configuration parameters
 */

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Actuator2Constants;

public class Actuator2 extends SubsystemBase {
    
    /**
     * SPARK MAX motor controller for the actuator.
     * 
     * Configured for basic voltage control only - no onboard PID.
     * Uses NEO brushless motor for efficient, high-torque actuation.
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
     * WPILib PID controller running on RoboRIO.
     * 
     * Calculates motor output based on position error.
     * Updated every robot periodic cycle (~20ms).
     */
    private final PIDController pidController;
    
    /**
     * Current setpoint position in rotations.
     * 
     * Target position for PID controller.
     * Updated whenever a new position is commanded.
     */
    private double currentSetpoint = 0.0;
    
    /**
     * Whether PID control is currently active.
     * 
     * When false, motor is in manual control mode.
     * When true, PID controller calculates motor output.
     */
    private boolean pidEnabled = false;
    
    /**
     * Creates a new Actuator2 subsystem.
     * 
     * Initializes motor controller with:
     * - Basic voltage control (no onboard PID)
     * - Current limits to protect motor and mechanism
     * - Brake mode for holding position
     * - Encoder configuration and zeroing
     * - WPILib PID controller setup
     */
    public Actuator2() {
        // Initialize SPARK MAX motor controller
        motor = new SparkMax(Actuator2Constants.kMotorCanId, MotorType.kBrushless);
        
        // Create configuration object
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Configure basic motor settings
        config.inverted(Actuator2Constants.kMotorInverted)
              .idleMode(IdleMode.kBrake); // Brake mode holds position when idle
        
        // Configure current limits
        config.smartCurrentLimit(Actuator2Constants.kCurrentLimit);
        
        // Configure encoder
        config.encoder.positionConversionFactor(Actuator2Constants.kPositionConversionFactor)
                      .velocityConversionFactor(Actuator2Constants.kVelocityConversionFactor);
        
        // Apply configuration to motor controller
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Get encoder reference
        encoder = motor.getEncoder();
        
        // Reset encoder to zero (assumes mechanism starts at home position)
        encoder.setPosition(0);
        
        // Initialize WPILib PID controller
        pidController = new PIDController(
            Actuator2Constants.kP,
            Actuator2Constants.kI,
            Actuator2Constants.kD
        );
        
        // Set PID output limits to match motor controller range
        pidController.setTolerance(Actuator2Constants.kPositionTolerance);
        
        // Initialize setpoint to current position
        currentSetpoint = encoder.getPosition();
    }
    
    /**
     * Sets the target position for PID control.
     * 
     * The actuator will move to this position using WPILib PID control.
     * Position is automatically clamped to safe range.
     * 
     * @param position Target position in rotations (or custom units if conversion factor set)
     */
    public void setPosition(double position) {
        // Clamp position to safe range
        double clampedPosition = Math.max(Actuator2Constants.kReverseSoftLimit,
                                         Math.min(Actuator2Constants.kForwardSoftLimit, position));
        
        // Update setpoint
        currentSetpoint = clampedPosition;
        pidController.setSetpoint(clampedPosition);
        
        // Enable PID control
        pidEnabled = true;
    }
    
    /**
     * Moves actuator with manual voltage control.
     * 
     * Used for joystick control. Speed is clamped and position limits are enforced.
     * Disables PID control while in manual mode.
     * 
     * @param speed Speed from -1.0 (full reverse) to 1.0 (full forward)
     */
    public void setManualSpeed(double speed) {
        // Disable PID control
        pidEnabled = false;
        
        // Clamp speed to valid range
        speed = Math.max(-1.0, Math.min(1.0, speed));
        
        // Enforce soft limits in manual mode
        double currentPosition = encoder.getPosition();
        if (currentPosition >= Actuator2Constants.kForwardSoftLimit && speed > 0) {
            speed = 0; // Stop at forward limit
        } else if (currentPosition <= Actuator2Constants.kReverseSoftLimit && speed < 0) {
            speed = 0; // Stop at reverse limit
        }
        
        // Apply voltage directly
        motor.set(speed);
        
        // Update setpoint to current position (for smooth transition back to auto)
        currentSetpoint = currentPosition;
    }
    
    /**
     * Stops the actuator motor.
     * 
     * Disables PID and stops motor. In brake mode, the motor will hold position.
     */
    public void stop() {
        pidEnabled = false;
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
     * Uses PID controller's built-in tolerance checking.
     * 
     * @return true if within tolerance of setpoint
     */
    public boolean atSetpoint() {
        return pidController.atSetpoint();
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
        pidController.reset();
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
    
    /**
     * Gets the PID controller for direct access (for tuning via dashboard).
     * 
     * @return The WPILib PIDController instance
     */
    public PIDController getPIDController() {
        return pidController;
    }
    
    @Override
    public void periodic() {
        // If PID is enabled, calculate and apply motor output
        if (pidEnabled) {
            double currentPosition = encoder.getPosition();
            
            // Calculate PID output
            double output = pidController.calculate(currentPosition);
            
            // Clamp output to valid range
            output = Math.max(Actuator2Constants.kMinOutput, 
                             Math.min(Actuator2Constants.kMaxOutput, output));
            
            // Apply calculated output to motor
            motor.set(output);
        }
        
        // Update telemetry to dashboard
        SmartDashboard.putNumber("Actuator2/Position", getPosition());
        SmartDashboard.putNumber("Actuator2/Velocity", getVelocity());
        SmartDashboard.putNumber("Actuator2/Setpoint", getSetpoint());
        SmartDashboard.putBoolean("Actuator2/At Setpoint", atSetpoint());
        SmartDashboard.putBoolean("Actuator2/PID Enabled", pidEnabled);
        SmartDashboard.putNumber("Actuator2/Current (A)", getCurrent());
        SmartDashboard.putNumber("Actuator2/Output %", getMotorOutput());
        SmartDashboard.putNumber("Actuator2/PID Error", pidController.getError());
    }
}
