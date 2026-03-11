// =============================================================================
// Shooter2.java — Flywheel Velocity PID Subsystem
// =============================================================================
//
// OVERVIEW
// --------
// Controls a dual-NEO Vortex (SparkFlex) flywheel and a Neo 550 (SparkMax) feeder
// belt via on-board closed-loop velocity control running at 1 kHz on each SPARK
// controller.  Tunable gains (F, P, I, D) and setpoints are exposed through
// SmartDashboard so they can be adjusted live without redeploying code.
//
// =============================================================================
// TUNING STRATEGY — Velocity PID on a Flywheel
// =============================================================================
//
// RECOMMENDED TOOLING (target vs. actual graphs)
// -----------------------------------------------
//  1. Shuffleboard (built into WPILib)
//       - Drag "Shooter/RPM" and "Shooter/TargetRPM" SmartDashboard entries onto
//         a tab and change their widget type to "Graph".
//       - Both signals appear on the same time-series plot, making overshoot and
//         steady-state error immediately visible.
//       - Open via: Driver Station → Shuffleboard (or run `shuffleboard` from a
//         terminal after deploying).
//
//  2. Glass (WPILib desktop tool)
//       - Provides a lightweight NetworkTables client with built-in plot windows.
//       - Run: `glass` from the WPILib command palette or start menu shortcut.
//       - Add entries under "NT" → drag to a "Plot" widget for live graphing.
//

// STEP-BY-STEP TUNING PROCEDURE
// ------------------------------
//  Phase 1 — Feedforward (F aka kV)  — eliminates steady-state error at speed
//    1. Set kP = 0, kI = 0, kD = 0.
//    2. Set kV to a rough estimate: kV ≈ 12.0 / maxFreeRPM  (≈ 0.0024 for 5000 RPM).
//    3. Command the target RPM and watch the graph.  Increase kV until the actual
//       RPM roughly tracks the target without oscillating.
//    4. A good kV means the motor reaches ~90–95 % of target on feedforward alone.
//
//  Phase 2 — Proportional (P)  — closes the remaining error
//    1. With a tuned kV, add a small kP (e.g. 0.0005).
//    2. Increase kP until the RPM reaches setpoint quickly with minimal overshoot
//       (< 5 % is typical for a flywheel).
//    3. If the motor oscillates or sounds rough, kP is too high — back off by 50 %.
//
//  Phase 3 — Derivative (D)  — damps overshoot (usually not needed for flywheels)
//    1. Only add kD if kP causes noticeable oscillation that kV alone cannot cure.
//    2. Start at kD ≈ 10 × kP; increase until oscillation damps.
//    3. Too much kD causes a high-frequency buzz — reduce if heard.
//
//  Phase 4 — Integral (I)  — corrects persistent steady-state error
//    1. Flywheels rarely need kI if kV and kP are well-tuned.
//    2. If a small constant offset remains, add kI ≈ 0.001 and watch for wind-up.
//    3. Use the integral zone (iZone) on the SPARK to limit accumulation.
//
//  Phase 5 — Validate under load
//    1. Fire a game piece and observe how quickly the RPM recovers.
//    2. Increase kP or kV slightly if recovery is sluggish; reduce if the motor
//       oscillates after the shot.
//
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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Shooter tuning
    private double distance     = 2.0;      // Distance from target (meters); replace with vision measurement
    private double targetRpm   = 3000.0;
    private double readyRpm    = 2980.0;   // Minimum RPM before feeder activates
    private double feederSpeed = 0.5;      // Feeder open-loop duty cycle [0, 1]
    private double maxOutput   = 0.85;     // Maximum closed-loop output [0, 1]


    // Closed-loop velocity PID gains (tune these on the physical robot)
    private double kP = 0.00025;
    private double kI = 0.000002;
    private double kD = 0.0064;

    // kV(aka kF) feedforward (volts per RPM). Set to ~1/maxRPM for a first estimate.
    private double kF = 0.0000016;//12/targetRpm;  // Tune this on the physical robot


    private final SparkMax feederMotor;
    private final SparkFlex shooterMotor1;
    private final SparkFlex shooterMotor2;
    private final SparkFlexConfig shooter1Config;
    private final Drivetrain drivetrain;

    // On-board closed-loop controller runs the PID at 1 kHz on the SPARK MAX
    // (vs. 50 Hz if run on the RoboRIO with WPILib PIDController)
    private final SparkClosedLoopController flywheelController;

    private boolean isShooterActive  = false;
    private double  currentFeederSpeed = 0.0;

    public Shooter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        feederMotor   = new SparkMax(ShooterConstants.FEEDER_CAN_ID,   MotorType.kBrushless);
        shooterMotor1 = new SparkFlex(ShooterConstants.SHOOTER1_CAN_ID, MotorType.kBrushless);
        shooterMotor2 = new SparkFlex(ShooterConstants.SHOOTER2_CAN_ID, MotorType.kBrushless);
        shooter1Config = new SparkFlexConfig();

        flywheelController = shooterMotor1.getClosedLoopController();

        // Push initial values to SmartDashboard
        SmartDashboard.putNumber("Shooter/F", kF);
        SmartDashboard.putNumber("Shooter/P", kP);
        SmartDashboard.putNumber("Shooter/I", kI);
        SmartDashboard.putNumber("Shooter/D", kD);
        SmartDashboard.putNumber("Shooter/MaxOutput", maxOutput);
        SmartDashboard.putNumber("Shooter/Distance", distance);
        SmartDashboard.putNumber("Shooter/ReadyRPM", readyRpm);
        SmartDashboard.putNumber("Shooter/FeederSpeed", feederSpeed);

        configureMotors();
    }

    private void configureMotors() {
        // --- Feeder motor ---
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);  // Hard-stop when commanded to 0

        // --- Primary flywheel motor (closed-loop velocity) ---
        // Reuse the instance variable shooter1Config
        shooter1Config
            .inverted(true)//set to true
            .idleMode(IdleMode.kCoast);  // Coast so the flywheel spins down naturally

        shooter1Config.encoder
            .velocityConversionFactor(1.0);  // RPM (native NEO units)

        shooter1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(0, maxOutput);  // Flywheel only spins forward

        shooter1Config.closedLoop.feedForward
            .kV(kF);  // Velocity feedforward (Volts per RPM); tune on robot

        // --- Follower flywheel motor ---
        // follow(id, inverted=true) → spins opposite direction to motor1,
        // which physically makes both wheels shoot in the same direction.
        SparkFlexConfig shooter2Config = new SparkFlexConfig();
        shooter2Config
            //.idleMode(IdleMode.kCoast)
            .follow(ShooterConstants.SHOOTER1_CAN_ID, true); //Invert to match direction of motor 1

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
        return shooterMotor1.getEncoder().getVelocity() >= readyRpm;
    }

    /** Spins up the flywheel to {@link #targetRpm} */
    public void activateShooter() {
        isShooterActive = true;
    }

    /** No-op; subsystem state is already initialized in the constructor */
    public void initialize() {}

    /** Coasts the flywheel to a stop */
    public void killShooter() {
        isShooterActive = false;
    }

    /** Runs the feeder belt at {@link #feederSpeed} */
    public void activateFeeder() {
        currentFeederSpeed = feederSpeed;
    }

    /** Stops the feeder belt */
    public void killFeeder() {
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
        final double MAX_RPM = 4500.0;         // RPM at maximum distance

        // Clamp distance to valid range
        double clampedDistance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distanceFromHub));

        // Linear interpolation
        double rpm = MIN_RPM + (clampedDistance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE) * (MAX_RPM - MIN_RPM);

        return rpm;
    }

    private Translation2d getAllianceHubCenter() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red
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

        // Publish telemetry to SmartDashboard / Shuffleboard
        SmartDashboard.putNumber("Shooter/RPM",     shooterMotor1.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter/Charged", isCharged());
        SmartDashboard.putBoolean("Shooter/Active",  isShooterActive);
    }

    private void updateTunables() {
        double f = SmartDashboard.getNumber("Shooter/F", kF);
        double p = SmartDashboard.getNumber("Shooter/P", kP);
        double i = SmartDashboard.getNumber("Shooter/I", kI);
        double d = SmartDashboard.getNumber("Shooter/D", kD);
        double max = SmartDashboard.getNumber("Shooter/MaxOutput", maxOutput);

        double hubdistance = getHubDistance();
        double ready = SmartDashboard.getNumber("Shooter/ReadyRPM", readyRpm);
        double feeder = SmartDashboard.getNumber("Shooter/FeederSpeed", feederSpeed);

        // Update local variables
        distance = hubdistance;
        SmartDashboard.putNumber("Shooter/Distance", distance);
        targetRpm = calculateRpmFromDistance(hubdistance);
        //readyRpm = ready;
        readyRpm=targetRpm * 0.99; // Set ready RPM to 98% of target RPM for a small buffer
        feederSpeed = feeder;

        // Check if PIDF or MaxOutput changed
        if (f != kF || p != kP || i != kI || d != kD || max != maxOutput) {
            kF = f;
            kP = p;
            kI = i;
            kD = d;
            maxOutput = max;

            kF=0.0000016; //tuned value.  not rpm dependent

            // Update configuration object
            shooter1Config.closedLoop
                .p(kP)
                .i(kI)
                .d(kD)
                .outputRange(0, maxOutput);
            shooter1Config.closedLoop.feedForward
                .kV(kF);

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
        if (isShooterActive) {
            // Delegate PID calculation to the SPARK MAX (runs at 1 kHz on-board)
            flywheelController.setSetpoint(targetRpm, ControlType.kVelocity);
        } else {
            shooterMotor1.set(0);
        }
        feederMotor.set(currentFeederSpeed);
    }
}
