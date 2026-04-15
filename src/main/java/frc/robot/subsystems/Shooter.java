// TUNING RULES OF THUMB
// ----------------------
//  • Always tune feedforward before feedback (F before P before D before I).
//  • Use SmartDashboard live-tuning (updateTunables) — no redeploy required.
//  • Keep maxOutput ≤ 0.85 until gains are stable to protect the mechanism.
//  • Once gains are finalized, change PersistMode to kPersistParameters and
//    commit the constants in code so they survive a power cycle.
//
// =============================================================================

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VortexMotorConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final SparkMax feederMotor;
    private final SparkFlex shooterMotor1;
    private final SparkFlex shooterMotor2;
    private final SparkFlexConfig shooter1Config;
    private final SparkFlexConfig shooter2Config;
    private final Drivetrain drivetrain;
    private final Timer shooterSpinTimer = new Timer();

    // On-board closed-loop controller runs the PID at 1 kHz on the SPARK MAX
    // (vs. 50 Hz if run on the RoboRIO with WPILib PIDController)
    private final SparkClosedLoopController flywheelController1;

    private static final double IDLE_RPM = 1500.0;
    private static final double VORTEX_SPEED_SHOT_TARGET_RPM = VortexMotorConstants.kFreeSpeedRpm * 0.95;
    private static final double VORTEX_SPEED_SHOT_READY_RPM = VORTEX_SPEED_SHOT_TARGET_RPM * 0.50;

    private boolean isShooterActive  = false;
    private boolean isVortexSpeedShotActive = false;
    private boolean isFeederReversed = false;
    private double  currentFeederSpeed = 0.0;

    public Shooter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        feederMotor   = new SparkMax(ShooterConstants.FEEDER_CAN_ID,   MotorType.kBrushless);
        shooterMotor1 = new SparkFlex(ShooterConstants.SHOOTER1_CAN_ID, MotorType.kBrushless);
        shooterMotor2 = new SparkFlex(ShooterConstants.SHOOTER2_CAN_ID, MotorType.kBrushless);
        shooter1Config = new SparkFlexConfig();
        shooter2Config = new SparkFlexConfig();

        flywheelController1 = shooterMotor1.getClosedLoopController();

        publishTunables();
        publishTelemetry();

        configureMotors();
    }

    private void publishTunables() {
        // SmartDashboard.putNumber("Shooter/Motor1/F", ShooterConstants.kF1);
        // SmartDashboard.putNumber("Shooter/Motor1/P", ShooterConstants.kP1);
        // SmartDashboard.putNumber("Shooter/Motor1/I", ShooterConstants.kI1);
        // SmartDashboard.putNumber("Shooter/Motor1/D", ShooterConstants.kD1);
        // SmartDashboard.putNumber("Shooter/Motor2/F", ShooterConstants.kF2);
        // SmartDashboard.putNumber("Shooter/Motor2/P", ShooterConstants.kP2);
        // SmartDashboard.putNumber("Shooter/Motor2/I", ShooterConstants.kI2);
        // SmartDashboard.putNumber("Shooter/Motor2/D", ShooterConstants.kD2);
        // SmartDashboard.putNumber("Shooter/MaxOutput", ShooterConstants.maxOutput);
        // SmartDashboard.putNumber("Shooter/ReadyRPM", ShooterConstants.readyRpm);
        // SmartDashboard.putNumber("Shooter/FeederSpeed", ShooterConstants.feederSpeed);
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber("Shooter/Distance", getHubDistance());
        SmartDashboard.putNumber("Shooter/TargetRPM", ShooterConstants.targetRpm);
        SmartDashboard.putNumber("Shooter/ReadyRPM", ShooterConstants.readyRpm);
        SmartDashboard.putNumber("Shooter/Shooter1RPM", shooterMotor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Shooter2RPM", shooterMotor2.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/AppliedFeederSpeed", currentFeederSpeed);
        SmartDashboard.putBoolean("Shooter/VortexSpeedShotActive", isVortexSpeedShotActive);
        SmartDashboard.putBoolean("Shooter/VortexSpeedShotReady", isVortexSpeedShotReady());

        Logger.recordOutput("Shooter/Distance", getHubDistance());
        Logger.recordOutput("Shooter/TargetRPM", ShooterConstants.targetRpm);
        Logger.recordOutput("Shooter/ReadyRPM", ShooterConstants.readyRpm);
        Logger.recordOutput("Shooter/Shooter1RPM", shooterMotor1.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Shooter2RPM", shooterMotor2.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/AppliedFeederSpeed", currentFeederSpeed);
        Logger.recordOutput("Shooter/VortexSpeedShotActive", isVortexSpeedShotActive);
        Logger.recordOutput("Shooter/RPM", shooterMotor1.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Charged", isCharged());
        Logger.recordOutput("Shooter/Active", isShooterActive);
        Logger.recordOutput("Shooter/Motor1CurrentAmps", shooterMotor1.getOutputCurrent());
        Logger.recordOutput("Shooter/Motor2CurrentAmps", shooterMotor2.getOutputCurrent());
        Logger.recordOutput("Shooter/FeederCurrentAmps", feederMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/RPMError", shooterMotor1.getEncoder().getVelocity() - ShooterConstants.targetRpm);
    }

    private void applyClosedLoopConfig(SparkFlexConfig config, double p, double i, double d, double f) {
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(p)
            .i(i)
            .d(d)
            .outputRange(0, ShooterConstants.maxOutput);

        config.closedLoop.feedForward.kV(f);
    }

    private void configureMotors() {
        // --- Feeder motor ---
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast);  // Hard-stop when commanded to 0

        // --- Primary flywheel motor (closed-loop velocity) ---
        // Reuse the instance variable shooter1Config
        shooter1Config
            .inverted(true)//set to true
            .idleMode(IdleMode.kCoast);  // Coast so the flywheel spins down naturally

        shooter1Config.encoder
            .velocityConversionFactor(1.0);  // RPM (native NEO units)

        applyClosedLoopConfig(
            shooter1Config,
            ShooterConstants.kP1,
            ShooterConstants.kI1,
            ShooterConstants.kD1,
            ShooterConstants.kF1);

        // follow(id, inverted=true) → spins opposite direction to motor1,
        // which physically makes both wheels shoot in the same direction.
        shooter2Config
            .follow(ShooterConstants.SHOOTER1_CAN_ID, true)
            .idleMode(IdleMode.kCoast);  // Coast so the flywheel spins down naturally

        shooter2Config.encoder
            .velocityConversionFactor(1.0);  // RPM (native NEO units)

        feederMotor.configure(
            feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        shooterMotor1.configure(
            shooter1Config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        shooterMotor2.configure(
            shooter2Config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    /** @return true when the flywheel has reached the ready RPM threshold */
    public boolean isCharged() {
        double currentVelocity1 = shooterMotor1.getEncoder().getVelocity();
        double allowedError = Math.abs(ShooterConstants.targetRpm - ShooterConstants.readyRpm);

        Boolean isCharged1 = Math.abs(currentVelocity1 - ShooterConstants.targetRpm) <= allowedError;
        return isCharged1;

    }

    /** Spins up the flywheel to {@link #targetRpm} */
    public void startShooter() {
        isShooterActive = true;
        isVortexSpeedShotActive = false;

        // Restore coast mode
        setShooterIdleMode(IdleMode.kCoast);

        shooterSpinTimer.reset();
        shooterSpinTimer.start();
    }

    /** Spins the shooter to 95% of NEO Vortex free speed and auto-runs the feeder at 80% of target RPM. */
    public void startVortexSpeedShot() {
        isShooterActive = false;
        isVortexSpeedShotActive = true;

        setShooterIdleMode(IdleMode.kCoast);
    }

    public boolean isVortexSpeedShotReady() {
        double currentVelocity1 = shooterMotor1.getEncoder().getVelocity();
        return currentVelocity1 >= VORTEX_SPEED_SHOT_READY_RPM;
    }

    /** No-op; subsystem state is already initialized in the constructor */
    public void initialize() {}

    /** Coasts the flywheel to a stop */
    public void stopShooter() {
        isShooterActive = false;
        isVortexSpeedShotActive = false;
        shooterSpinTimer.stop();
    }
    public void killShooter() {
        isShooterActive = false;
        isVortexSpeedShotActive = false;

        // Switch to brake mode and stop motors
        setShooterIdleMode(IdleMode.kBrake);
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    private void setShooterIdleMode(IdleMode mode) {
        // Only change idle mode; reuse the existing config
        shooter1Config.idleMode(mode);
        shooter2Config.idleMode(mode);

        shooterMotor1.configure(shooter1Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterMotor2.configure(shooter2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Runs the feeder belt at {@link #feederSpeed} */
    public void startFeeder() {
        currentFeederSpeed = ShooterConstants.feederSpeed;
    }

    public void toggleReverseFeeder() {
        currentFeederSpeed = -ShooterConstants.feederSpeed;
    }

    /**
     * Runs the feeder in reverse regardless of shooter state.
     * Safe to call while the shooter is stopped — does not interact with
     * {@link #isCharged()} since that gate only applies when
     * {@code isShooterActive} is true.
     */
    public void reverseFeeder() {
        isFeederReversed = true;
    }

    /** Stops the feeder that was running in reverse via {@link #reverseFeeder()}. */
    public void stopReverseFeeder() {
        isFeederReversed = false;
    }

    /** Stops the feeder belt */
    public void stopFeeder() {
        currentFeederSpeed = 0.0;
    }

    /**
     * Calculates the target RPM based on the distance from the center of the hub.
     * 
     * @param distanceFromHub the distance in meters from the center of the hub
     * @return the target RPM for the shooter flywheel
     */
    public double calculateRpmFromDistance(double distanceFromHub) {
        // Linear interpolation model: RPM increases with distance
        // Adjust these constants based on your shooter's ballistics
        final double MIN_DISTANCE = 1.0;      // Minimum shooting distance (meters)
        final double MAX_DISTANCE = 3.0;     // Maximum shooting distance (meters)
        final double MIN_RPM = 1500.0;         // RPM at minimum distance
        final double MAX_RPM = 3800.0 * 0.8 ;         // RPM at maximum distance

        // Clamp distance to valid range
        double clampedDistance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distanceFromHub));

        // Linear interpolation
        double rpm = (MIN_RPM + (clampedDistance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE) * (MAX_RPM - MIN_RPM)) * 1.00;

        return rpm;
    }

    private Translation2d getAllianceHubCenter() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Red)
            ? NavigationConstants.kRedAllianceHubCenter
            : NavigationConstants.kBlueAllianceHubCenter;
    }

    private double getHubDistance() {
        return drivetrain.getPose().getTranslation().getDistance(getAllianceHubCenter());
    }

    @Override
    public void periodic() {
        updateTunables();
        updateMotors();
        publishTunables();
        publishTelemetry();

        // Publish telemetry to SmartDashboard / Shuffleboard
        SmartDashboard.putNumber("Shooter/RPM",     shooterMotor1.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter/Charged", isCharged());
        SmartDashboard.putBoolean("Shooter/Active",  isShooterActive);
    }

    private void updateTunables() {
        double motor1F = SmartDashboard.getNumber("Shooter/Motor1/F", ShooterConstants.kF1);
        double motor1P = SmartDashboard.getNumber("Shooter/Motor1/P", ShooterConstants.kP1);
        double motor1I = SmartDashboard.getNumber("Shooter/Motor1/I", ShooterConstants.kI1);
        double motor1D = SmartDashboard.getNumber("Shooter/Motor1/D", ShooterConstants.kD1);
        double max = SmartDashboard.getNumber("Shooter/MaxOutput", ShooterConstants.maxOutput);
        double readyRpm = SmartDashboard.getNumber("Shooter/ReadyRPM", ShooterConstants.readyRpm);
        double feederSpeed = SmartDashboard.getNumber("Shooter/FeederSpeed", ShooterConstants.feederSpeed);

        double hubdistance = getHubDistance();
        ShooterConstants.targetRpm = calculateRpmFromDistance(hubdistance);

        ShooterConstants.readyRpm = readyRpm;
        ShooterConstants.feederSpeed = feederSpeed;        

        // Check if PIDF or MaxOutput changed
        if (motor1F != ShooterConstants.kF1
            || motor1P != ShooterConstants.kP1
            || motor1I != ShooterConstants.kI1
            || motor1D != ShooterConstants.kD1
            || max != ShooterConstants.maxOutput) {
            ShooterConstants.kF1 = motor1F;
            ShooterConstants.kP1 = motor1P;
            ShooterConstants.kI1 = motor1I;
            ShooterConstants.kD1 = motor1D;
            ShooterConstants.maxOutput = max;

            applyClosedLoopConfig(
                shooter1Config,
                ShooterConstants.kP1,
                ShooterConstants.kI1,
                ShooterConstants.kD1,
                ShooterConstants.kF1);

            // Re-apply configuration to the motor controller
            // We use kNoResetSafeParameters to avoid resetting other settings
            // We use kNoPersistParameters to avoid wearing out flash memory during tuning
            shooterMotor1.configure(
                shooter1Config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        }
    }

    private void updateMotors() {
        double feederOutput = 0.0;

        if (!DriverStation.isEnabled()) {
            shooterMotor1.set(0);
            shooterMotor2.set(0);
            isFeederReversed = false;
        } else if (isVortexSpeedShotActive) {
            flywheelController1.setSetpoint(VORTEX_SPEED_SHOT_TARGET_RPM, ControlType.kVelocity);
            if (isVortexSpeedShotReady()) {
                feederOutput = ShooterConstants.feederSpeed;
            }
        } else if (isShooterActive) {
            // Delegate PID calculation to the SPARK MAX (runs at 1 kHz on-board)
            flywheelController1.setSetpoint(ShooterConstants.targetRpm, ControlType.kVelocity);
            if (isCharged() && shooterSpinTimer.hasElapsed(0.5)) {  // Add a short delay after reaching target RPM
                feederOutput = currentFeederSpeed;
            }
        } else if (isFeederReversed) {
            // Direct reverse without shooter active — isCharged() is not involved here
            flywheelController1.setSetpoint(IDLE_RPM, ControlType.kVelocity);
            feederOutput = -ShooterConstants.feederSpeed;
        } else {
            flywheelController1.setSetpoint(IDLE_RPM, ControlType.kVelocity);
        }

        feederMotor.set(feederOutput);
    }
}
