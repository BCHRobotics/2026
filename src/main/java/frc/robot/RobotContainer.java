package frc.robot;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.drivetrain.PointRearToAllianceHubCommand;
import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.drivetrain.VisionTuningPath;
import frc.robot.commands.drivetrain.ZeroHeadingCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.VortexSpeedShotCommand;
import frc.robot.commands.ballintake.CalibrateBallIntakeCommand;
import frc.robot.commands.ballintake.HoldBallIntakeExtendCommand;
import frc.robot.commands.ballintake.ToggleBallIntakeExtendCommand;
import frc.robot.commands.ballintake.ReverseBallIntakeAndFeederCommand;
import frc.robot.commands.climber.CalibrateClimberCommand;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

public class RobotContainer {
    // Subsystems
    private final Drivetrain robotDrive = new Drivetrain();
    private final BallIntake m_ballIntake = new BallIntake();
    private final Climber climber = new Climber();
    private final Shooter m_shooter = new Shooter(robotDrive);

    // Vision subsystem for AprilTag detection and pose estimation.
    private final Vision vision = new Vision(robotDrive);

    // Encapsulated match timer logic
    private final MatchTimer matchTimer = new MatchTimer();
    // Make hubTimerTrigger a field so we can sample its state elsewhere
    private Trigger hubTimerTrigger;
    // Teleop and shift toggle state
    private boolean teleopEnabled = false;
    private boolean shiftToggle = false; // latched: toggled once per touchpad press during teleop
    
    // Controllers
    CommandPS5Controller driverPS5;
    CommandXboxController driverXbox;
    // Operator controller (port 1) — always PS5, used for intake controls
    CommandPS5Controller operatorPS5;
    CommandXboxController operatorXbox;

    // Autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Field pose chooser — selects where the robot drives when the pose-nav button
    // is held
    private final SendableChooser<Pose2d> fieldPoseChooser = new SendableChooser<>();
    private final SendableChooser<Pose2d> climbStartPoseChooser = new SendableChooser<>();
    private final SendableChooser<PIDConstants> ppTranslationPidChooser = new SendableChooser<>();
    private final SendableChooser<PIDConstants> ppRotationPidChooser = new SendableChooser<>();
    private final SendableChooser<Boolean> intakeOverrideCalibrationLimitsChooser = new SendableChooser<>();

    // The container for the robot, initializing everything and setting up the
    // controller chooser
    public RobotContainer() {
        // Initialize Driver Controller based on Type
        if (OIConstants.kDriverControllerType == OIConstants.ControllerType.PS5) {
            driverPS5 = new CommandPS5Controller(OIConstants.kMainControllerPort);
        } else {
            driverXbox = new CommandXboxController(OIConstants.kMainControllerPort);
        }

        if (OIConstants.kOperatorControllerType == OIConstants.ControllerType.PS5) {
            operatorPS5 = new CommandPS5Controller(OIConstants.kOperatorControllerPort);
        } else {
            operatorXbox = new CommandXboxController(OIConstants.kOperatorControllerPort);
        }

        // Start data logging for AdvantageScope
        DataLogManager.start();

        // Connect Vision subsystem to Drivetrain for diagnostics
        robotDrive.setVision(vision);
        configurePathPlannerLogging();

    
        // Configure default commands
        configureDefaultCommands();

        // Configure button bindings
        configureBindings();

        // Configure field pose chooser
        configureClimbStartPoseChooser();
        configurePathPlannerPidChoosers();
        configureIntakeOverrideChooser();
        configureVisionTuning();
        configureDashboardCommands();
        registerPathPlannerCommands();

        // Auto Paths with Climb 
        autoChooser.addOption("OSCRT_Auto", new PathPlannerAuto("OSCRT_Auto"));
        autoChooser.addOption("OSCRB_Auto", new PathPlannerAuto("OSCRB_Auto"));
        autoChooser.addOption("OSCRM_Auto", new PathPlannerAuto("OSCRM_Auto"));
        autoChooser.addOption("S8CLT_Auto", new PathPlannerAuto("S8CLT_Auto"));
        autoChooser.addOption("S8CRT_Auto", new PathPlannerAuto("S8CRT_Auto"));
        autoChooser.addOption("S8CRM_Auto", new PathPlannerAuto("S8CRM_Auto"));

        // Auto Paths without Climb (only shoot)

        autoChooser.addOption("DSL_Auto", new PathPlannerAuto("DSL_Auto"));
        autoChooser.addOption("DSCLB_Auto", new PathPlannerAuto("DSCLB_Auto"));
        // autoChooser.addOption("DSM_Auto", new PathPlannerAuto("DSM_Auto"));
        // autoChooser.addOption("DSOSL_Auto", new PathPlannerAuto("DSOSL_Auto"));
        // autoChooser.addOption("DSOSM_Auto", new PathPlannerAuto("DSOSM_Auto"));
        // autoChooser.addOption("OSDSR_Auto", new PathPlannerAuto("OSDSR_Auto"));
        autoChooser.addOption("OSRT_Auto", new PathPlannerAuto("OSRT_Auto"));
        autoChooser.addOption("OSRB_Auto", new PathPlannerAuto("OSRB_Auto"));
        autoChooser.addOption("OSM_Auto", new PathPlannerAuto("OSM_Auto"));
        autoChooser.addOption("S8L_Auto", new PathPlannerAuto("S8L_Auto"));
        autoChooser.addOption("S8M_Auto", new PathPlannerAuto("S8M_Auto"));
        autoChooser.addOption("S8R_Auto", new PathPlannerAuto("S8R_Auto"));

        // Auto paths in neutral zone with Climb
        autoChooser.addOption("NBSCRT_Auto", new PathPlannerAuto("NBSCRT_Auto"));
        autoChooser.addOption("NTSOSRB_Auto", new PathPlannerAuto("NTSOSRB_Auto"));
        autoChooser.addOption("NRNBSCLRT_Auto ", new PathPlannerAuto("NRNBSCLRT_Auto "));
        
        // Safety null option
        autoChooser.addOption("None", Commands.none());

        // Put the chooser on SmartDashboard for driver selection
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Populates and publishes the field pose chooser on SmartDashboard.
     *
     * Coordinates are blue-alliance-relative. GoToPositionRelativeCommand mirrors
     * them automatically when the DriverStation reports red alliance:
     * red_x = fieldLength - x, red_y = fieldWidth - y, red_heading = 180 - heading
     */
    private void configureClimbStartPoseChooser() {
        // These poses are predefined constants so the team can place the robot at known climb
        // Select positions from SmartDashboard before running the command.
        climbStartPoseChooser.setDefaultOption("Blue Left", ClimbConstants.kBlueLeftStartPose);
        climbStartPoseChooser.addOption("Blue Right", ClimbConstants.kBlueRightStartPose);
        climbStartPoseChooser.addOption("Red Left", ClimbConstants.kRedLeftStartPose);
        climbStartPoseChooser.addOption("Red Right", ClimbConstants.kRedRightStartPose);
        climbStartPoseChooser.addOption("PracticePose", ClimbConstants.PracticePose);

        SmartDashboard.putData("Climb Start Pose", climbStartPoseChooser);
    }

    /**
     * Configures Shuffleboard choosers for PathPlanner translation/rotation PID presets.
     */
    private void configurePathPlannerPidChoosers() {
        ppTranslationPidChooser.setDefaultOption(
                "Default (2.0, 1.0, 0.0)",
            AutoConstants.translationConstants
        );
        ppTranslationPidChooser.addOption("Soft (1.2, 0.0, 0.0)", new PIDConstants(1.2, 0.0, 0.0));
        ppTranslationPidChooser.addOption("Aggressive (2.0, 0.0, 0.0)", new PIDConstants(2.0, 0.0, 0.0));

        ppRotationPidChooser.setDefaultOption(
                "Default (1.0, 0.0, 0.0)",
            AutoConstants.rotationConstants
        );
        ppRotationPidChooser.addOption("Soft (0.6, 0.0, 0.0)", new PIDConstants(0.6, 0.0, 0.0));
        ppRotationPidChooser.addOption("Aggressive (1.6, 0.0, 0.0)", new PIDConstants(1.6, 0.0, 0.0));
    }

    private void configureIntakeOverrideChooser() {
        intakeOverrideCalibrationLimitsChooser.setDefaultOption("Limits Enabled", false);
        intakeOverrideCalibrationLimitsChooser.addOption("Override Calibration And Limits", true);
        SmartDashboard.putData("Intake Extend Override", intakeOverrideCalibrationLimitsChooser);
    }

    private void configureDashboardCommands() {
        // putData publishes a clickable command button to SmartDashboard.
        SmartDashboard.putData("Climb Command", new ClimbCommand(robotDrive, climber, this::getSelectedClimbStartPose));
        SmartDashboard.putData(
            "Intake Hold Extend",
            new HoldBallIntakeExtendCommand(m_ballIntake, 1.0, this::isIntakeOverrideCalibrationAndLimitsEnabled));
        SmartDashboard.putData(
            "Intake Hold Retract",
            new HoldBallIntakeExtendCommand(m_ballIntake, -1.0, this::isIntakeOverrideCalibrationAndLimitsEnabled));
    }

    private boolean isIntakeOverrideCalibrationAndLimitsEnabled() {
        Boolean selected = intakeOverrideCalibrationLimitsChooser.getSelected();
        return selected != null && selected;
    }

    private void registerPathPlannerCommands() {
        NamedCommands.registerCommand(
                "climber on",
                new ClimbCommand(robotDrive, climber, this::getSelectedClimbStartPose));
        NamedCommands.registerCommand(
                "shooter on",
                Commands.runOnce(() -> {
                    m_shooter.startShooter();
                    m_shooter.startFeeder();
                }, m_shooter));
        NamedCommands.registerCommand(
                "shooter off",
                Commands.runOnce(() -> {
                    m_shooter.stopShooter();
                    m_shooter.stopFeeder();
                }, m_shooter));
        NamedCommands.registerCommand(
                "intake on",
                Commands.runOnce(() -> {
                    m_ballIntake.moveToExtendedPosition();
                    m_ballIntake.run();
                }, m_ballIntake));
        NamedCommands.registerCommand(
                "intake off",
                Commands.runOnce(() -> {
                    m_ballIntake.moveToRetractedPosition();
                    m_ballIntake.stopRun();
                }, m_ballIntake));
        NamedCommands.registerCommand(
                "jiggle on", 
                Commands.runOnce(() ->  {
                    m_ballIntake.JiggleIntake();
                }, m_ballIntake));
        NamedCommands.registerCommand(
                "reverse on", 
                new ReverseBallIntakeAndFeederCommand(m_ballIntake, m_shooter)
                );
        NamedCommands.registerCommand(
                "reverse off", 
                Commands.runOnce(() -> {
                    m_ballIntake.stopReverseRun();
                    m_shooter.stopReverseFeeder();
                }, m_shooter, m_ballIntake));
    }

    private Pose2d getSelectedClimbStartPose() {
        Pose2d selectedPose = climbStartPoseChooser.getSelected();
        return selectedPose != null ? selectedPose : ClimbConstants.kBlueLeftStartPose;
    }

    private void configureVisionTuning() {
        SmartDashboard.putData("VisionTuningPath", new VisionTuningPath(robotDrive));
    }

    private void configurePathPlannerLogging() {
        PathPlannerLogging.setLogActivePathCallback(activePath ->
            Logger.recordOutput("Auto/ActivePath", activePath.toArray(new Pose2d[0]))
        );
        PathPlannerLogging.setLogTargetPoseCallback(targetPose -> {
            Logger.recordOutput("Auto/TargetPose", targetPose);
            Logger.recordOutput("Auto/TargetPose3d", new edu.wpi.first.math.geometry.Pose3d(targetPose));
        });
    }

    /**
     * Configures default commands for subsystems.
     * 
     * Default commands run continuously when no other command requiring
     * the subsystem is scheduled. This is where we set up teleop driving.
     */
    private void configureDefaultCommands() {
        // ========== Drivetrain Default Command: Teleop Drive ==========
        // Left stick controls translation (X/Y), right stick X controls rotation
        DoubleSupplier leftY, leftX, rightX;

        if (driverPS5 != null) {
            leftY = () -> -driverPS5.getLeftY() *0.6;
            leftX = () -> -driverPS5.getLeftX() *0.6;
            rightX = () -> -driverPS5.getRightX() *0.6;
        } else {
            leftY = () -> -driverXbox.getLeftY() *0.6;
            leftX = () -> -driverXbox.getLeftX() *0.6;
            rightX = () -> -driverXbox.getRightX() *0.6;
        }

        robotDrive.setDefaultCommand(
                new TeleopDriveCommand(
                        robotDrive,
                        leftY, // Forward/backward (inverted)
                        leftX, // Left/right (inverted)
                        rightX, // Rotation (inverted)
                        OIConstants.kFieldRelative, // Field-relative driving
                        OIConstants.kRateLimited // Enable slew rate limiting
                ));

        // CRITICAL: Set max speed for drivetrain (required for movement)
        robotDrive.setSpeedPercent();
    }

    // Configures button and trigger bindings for controllers.
    private void configureBindings() {

    Trigger pointRearToHub, intakeToggle, zeroHeading, extendToggle, shootTrigger, turboSpeedTrigger;
    Trigger killshooter, killIntake, climberExtend, climberRetract, vortexSpeedShot, jiggleIntake, calibrateIntake, holdIntakeExtend, holdIntakeRetract, reverseIntakeAndFeeder;
        DoubleSupplier leftY, leftX;

        if (driverPS5 != null) {
            pointRearToHub = driverPS5.square();
            intakeToggle = driverPS5.circle();
            zeroHeading = driverPS5.triangle();
            extendToggle = driverPS5.cross();
            turboSpeedTrigger = driverPS5.R1();
            shootTrigger = driverPS5.R2().or(driverPS5.L2());

            leftY = () -> -driverPS5.getLeftY();
            leftX = () -> -driverPS5.getLeftX();
        } else {
            pointRearToHub = driverXbox.x();
            intakeToggle = driverXbox.b();
            zeroHeading = driverXbox.y();
            extendToggle = driverXbox.a();
            turboSpeedTrigger = driverXbox.rightBumper();
            shootTrigger = driverXbox.rightTrigger().or(driverXbox.leftTrigger());;

            leftY = () -> -driverXbox.getLeftY();
            leftX = () -> -driverXbox.getLeftX();
        }

        if (operatorPS5 !=null) {
            killshooter = operatorPS5.square();
            killIntake = operatorPS5.circle();
            climberExtend = operatorPS5.triangle();
            climberRetract = operatorPS5.cross();
            vortexSpeedShot = operatorPS5.povLeft();
            jiggleIntake = operatorPS5.L2();
            calibrateIntake = operatorPS5.R1();
            holdIntakeExtend = operatorPS5.povUp();
            holdIntakeRetract = operatorPS5.povDown();
            reverseIntakeAndFeeder = operatorPS5.R2();
            hubTimerTrigger = operatorPS5.touchpad();
        } else {
            killshooter = operatorXbox.x();
            killIntake = operatorXbox.b();
            climberExtend = operatorXbox.y();
            climberRetract = operatorXbox.a();
            vortexSpeedShot = operatorXbox.povLeft();
            jiggleIntake = operatorXbox.leftTrigger();
            calibrateIntake = operatorXbox.rightBumper();
            holdIntakeExtend = operatorXbox.povUp();
            holdIntakeRetract = operatorXbox.povDown();
            reverseIntakeAndFeeder = operatorXbox.rightTrigger();
            hubTimerTrigger = operatorXbox.leftBumper();
        }

        // Toggle the shift selection on a single press, but only when teleop is enabled
        // and the press happens during TRANSITION (so the selection is decided before shifts).
        hubTimerTrigger.onTrue(
            Commands.runOnce(() -> {
                // When pressed during TRANSITION, select SHIFT1 & SHIFT3.
                if (teleopEnabled && matchTimer.isInTransitionPhase()) {
                    shiftToggle = true;
                }
            })
        );

        // Main controller commands

        zeroHeading.onTrue(new ZeroHeadingCommand(robotDrive));
        intakeToggle.onTrue(Commands.runOnce(m_ballIntake::toggleRun, m_ballIntake));
        extendToggle.onTrue(new ToggleBallIntakeExtendCommand(m_ballIntake));

        pointRearToHub.whileTrue(new PointRearToAllianceHubCommand(robotDrive));
        shootTrigger.whileTrue(new ShootCommand(m_shooter));
        turboSpeedTrigger.whileTrue(
            Commands.startEnd(
                robotDrive::enableTurboSpeed,
                robotDrive::disableTurboSpeed
            ));


        // Operator controller commands
        killshooter.onTrue(Commands.runOnce(m_shooter::killShooter, m_shooter));
        killIntake.onTrue(Commands.runOnce(m_ballIntake::stopRun, m_ballIntake));
        jiggleIntake.onTrue(Commands.runOnce(m_ballIntake::JiggleIntake, m_ballIntake));
        calibrateIntake.onTrue(new CalibrateBallIntakeCommand(m_ballIntake));
        holdIntakeExtend.whileTrue(
            new HoldBallIntakeExtendCommand(
                m_ballIntake,
                1.0,
                this::isIntakeOverrideCalibrationAndLimitsEnabled));

        holdIntakeRetract.whileTrue(
            new HoldBallIntakeExtendCommand(
                m_ballIntake,
                -1.0,
                this::isIntakeOverrideCalibrationAndLimitsEnabled));

        climberExtend.whileTrue(Commands.startEnd(climber::extendClimber, climber::stop, climber));
        climberRetract.whileTrue(Commands.startEnd(climber::retractClimber, climber::stop, climber));
        vortexSpeedShot.whileTrue(new VortexSpeedShotCommand(m_shooter));
        reverseIntakeAndFeeder.whileTrue(new ReverseBallIntakeAndFeederCommand(m_ballIntake, m_shooter));
    }

    /**
     * Returns the command to run during autonomous period.
     * 
     * @return the autonomous command selected from dashboard
     */
    public Command getAutonomousCommand() {
        PIDConstants translationConstants = ppTranslationPidChooser.getSelected();
        PIDConstants rotationConstants = ppRotationPidChooser.getSelected();
        if (translationConstants == null) {
            translationConstants = AutoConstants.translationConstants;
        }
        if (rotationConstants == null) {
            rotationConstants = AutoConstants.rotationConstants;
        }
        robotDrive.configureAutoBuilder(translationConstants, rotationConstants);

        Command selectedAuto = autoChooser.getSelected();
        if (selectedAuto == null) {
            selectedAuto = new InstantCommand();
        }

        return new CalibrateBallIntakeCommand(m_ballIntake)
                //.withTimeout(2.0) // Safety timeout for calibration - only use if necessary
                .andThen(new ConditionalCommand(
                        Commands.none(),
                        new PrintCommand("BallIntake calibration failed."),
                        m_ballIntake::isCalibrated))
                .andThen(new CalibrateClimberCommand(climber))
                .andThen(new ConditionalCommand(
                        Commands.none(),
                        new PrintCommand("Climber calibration failed."),
                        climber::isCalibrated))
                .andThen(selectedAuto);

    }

    public Command getTeleopInitCommand() {
        return new ZeroHeadingCommand(robotDrive)
            .andThen(
                new ConditionalCommand(
                    Commands.none(),
                    new CalibrateBallIntakeCommand(m_ballIntake),
                    m_ballIntake::isCalibrated
                )
            )
            .andThen(
                new ConditionalCommand(
                    Commands.none(),
                    new CalibrateClimberCommand(climber),
                    climber::isCalibrated
                )
            );
    }

    /**
     * Start the match timer if it is not already running.
     * This is public so Robot mode init methods can start the timer automatically.
     */
    public void startMatchTimer() {
        // Start AUTO phase from beginning
        matchTimer.start();
    }

    /** Start the timer at the beginning of TRANSITION (used when teleop begins). */
    public void startMatchTimerTransition() {
        matchTimer.startTransition();
    }

    /** Stop the match timer (used when disabled). */
    public void stopMatchTimer() {
        matchTimer.stop();
    }

    /**
     * Convenience API: start the autonomous timer (AUTO phase).
     */
    public void startAutonomousTimer() {
        startMatchTimer();
    }

    /**
     * Convenience API: start the teleop timer beginning at TRANSITION.
     */
    public void startTeleopTimer() {
        startMatchTimerTransition();
    }

    /** Set whether the robot is in teleop mode (enables the shift toggle input). */
    public void setTeleopEnabled(boolean enabled) {
        teleopEnabled = enabled;
        if (!enabled) {
            // reset latch when leaving teleop (optional)
            // shiftToggle = false;
        }
    }

    public void updateTimerDashboard() {
        // Pass the latched shift toggle into the MatchTimer so shifts use the toggled mapping.
        matchTimer.updateDashboard(shiftToggle);
    }
}