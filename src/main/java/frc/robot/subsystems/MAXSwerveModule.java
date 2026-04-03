/*
 * Normally I'd remove the m_ format, but since this is REV Robotics' code (not ours) so I'm leaving it
 */

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Configs;

public class MAXSwerveModule {
  public final SparkFlex m_drivingSpark;
  public final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Simulation
  private SparkSim m_drivingSparkSim;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    if (RobotBase.isSimulation()) {
      m_drivingSparkSim = new SparkSim(m_drivingSpark, DCMotor.getNeoVortex(1));
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
  
  /**
   * Gets the current draw of the driving motor.
   * 
   * @return Current in amps
   */
  public double getDriveCurrent() {
    return m_drivingSpark.getOutputCurrent();
  }
  
  /**
   * Gets the current draw of the turning motor.
   * 
   * @return Current in amps
   */
  public double getTurnCurrent() {
    return m_turningSpark.getOutputCurrent();
  }

  // Simulation: tracks module state directly since AbsoluteEncoder can't be driven by SparkSim
  private double simDrivePositionMeters = 0.0;
  private double simDriveVelocityMps = 0.0;
  private double simTurnPositionRad = 0.0;

  /**
   * Advances the simulated module state by dt seconds.
   * Bypasses the closed-loop controllers and directly tracks the desired state,
   * which is the standard approach for swerve simulation since the absolute
   * turning encoder cannot be driven through SparkSim.
   *
   * @param dt Loop period in seconds (typically 0.02)
   */
  public void simulationUpdate(double dt) {
    // Drive: integrate desired velocity into position
    simDriveVelocityMps = m_desiredState.speedMetersPerSecond;
    simDrivePositionMeters += simDriveVelocityMps * dt;

    // Turn: instantly snap to desired angle (perfect sim steering)
    simTurnPositionRad = m_desiredState.angle.getRadians() + m_chassisAngularOffset;

    // Push drive values into SparkFlex sim so getVelocity()/getPosition() work
    m_drivingSparkSim.iterate(simDriveVelocityMps, 12.0, dt);
  }

  /**
   * Returns the current simulated state of the module.
   * Only call this in simulation — in real mode use getState().
   */
  public SwerveModuleState getSimState() {
    return new SwerveModuleState(
        simDriveVelocityMps,
        new Rotation2d(simTurnPositionRad - m_chassisAngularOffset));
  }

  /**
   * Returns the current simulated position of the module.
   * Only call this in simulation — in real mode use getPosition().
   */
  public SwerveModulePosition getSimPosition() {
    return new SwerveModulePosition(
        simDrivePositionMeters,
        new Rotation2d(simTurnPositionRad - m_chassisAngularOffset));
  }
}
