package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
  private final SparkMax climbMotor;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;

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

  public void runClimber() {
    climbMotor.set(ClimbConstants.kClimbSpeed);
  }

  public void stop() {
    climbMotor.set(0.0);
  }

  public boolean isLimitSwitchPressed() {
    // This lets the team swap which limit input is used from Constants without changing code.
    return ClimbConstants.kUseForwardLimitSwitch
        ? forwardLimitSwitch.isPressed()
        : reverseLimitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    // Publish the important climber signals so students can debug the mechanism from the dashboard.
    SmartDashboard.putBoolean("Climber/LimitPressed", isLimitSwitchPressed());
    SmartDashboard.putNumber("Climber/Current", climbMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/AppliedOutput", climbMotor.getAppliedOutput());
  }
}