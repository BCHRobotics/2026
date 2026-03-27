package frc.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.drivetrain.PointRearToAllianceHubCommand;
import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.drivetrain.VisionTuningPath;
import frc.robot.commands.drivetrain.ZeroHeadingCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.VortexSpeedShotCommand;
import frc.robot.commands.ballintake.CalibrateBallIntakeCommand;
import frc.robot.commands.ballintake.ToggleBallIntakeExtendCommand;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

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

    
        // Configure default commands
        configureDefaultCommands();

        // Configure button bindings
        configureBindings();

        // Configure field pose chooser
        configureFieldPoseChooser();
        configureClimbStartPoseChooser();
        configurePathPlannerPidChoosers();
        configureVisionTuning();
        configureDashboardCommands();
        registerPathPlannerCommands();

        // Auto Paths with Climb 
        // autoChooser.addOption("Climber-1_Auto", new PathPlannerAuto("Climber-1_Auto"));
        // autoChooser.addOption("Climber-2_Auto", new PathPlannerAuto("Climber-2_Auto"));
        // autoChooser.addOption("Climber-3_Auto", new PathPlannerAuto("Climber-3_Auto"));
        // autoChooser.addOption("Climber-4_Auto", new PathPlannerAuto("Climber-4_Auto"));
        // autoChooser.addOption("Climber-5_Auto", new PathPlannerAuto("Climber-5_Auto"));
        // autoChooser.addOption("Climber-6_Auto", new PathPlannerAuto("Climber-6_Auto"));
        // autoChooser.addOption("Climber-7_Auto", new PathPlannerAuto("Climber-7_Auto"));
        // autoChooser.addOption("Climber-8_Auto", new PathPlannerAuto("Climber-8_Auto"));
        // autoChooser.addOption("Climber-9_Auto", new PathPlannerAuto("Climber-9_Auto"));
        // autoChooser.addOption("Climber-10_Auto", new PathPlannerAuto("Climber-10_Auto"));

        // Auto Paths without Climb (only shoot)
        autoChooser.addOption("DSL_Auto", new PathPlannerAuto("DSL_Auto"));
        autoChooser.addOption("DSM_Auto", new PathPlannerAuto("DSM_Auto"));
        autoChooser.addOption("DSOSL_Auto", new PathPlannerAuto("DSOSL_Auto"));
        autoChooser.addOption("DSOSM_Auto", new PathPlannerAuto("DSOSM_Auto"));
        autoChooser.addOption("OSDSR_Auto", new PathPlannerAuto("OSDSR_Auto"));
        autoChooser.addOption("OSR_Auto", new PathPlannerAuto("OSR_Auto"));
        autoChooser.addOption("S8L_Auto", new PathPlannerAuto("S8L_Auto"));
        autoChooser.addOption("S8M_Auto", new PathPlannerAuto("S8M_Auto"));
        autoChooser.addOption("S8R_Auto", new PathPlannerAuto("S8R_Auto"));
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
    private void configureFieldPoseChooser() {
        // Climber positions (final waypoints from PathPlanner paths, blue alliance)
        fieldPoseChooser.setDefaultOption("Climber 1", new Pose2d(1.037, 2.800, Rotation2d.fromDegrees(-90.0)));
        fieldPoseChooser.addOption("Climber 2", new Pose2d(1.061, 4.600, Rotation2d.fromDegrees(90.0)));
        fieldPoseChooser.addOption("Climber 3", new Pose2d(1.037, 2.800, Rotation2d.fromDegrees(-90.0)));
        fieldPoseChooser.addOption("Climber 4", new Pose2d(1.061, 4.600, Rotation2d.fromDegrees(90.0)));

        SmartDashboard.putData("Field Pose", fieldPoseChooser);
    }

    private void configureClimbStartPoseChooser() {
        // These poses are predefined constants so the team can place the robot at known climb
        // Select positions from SmartDashboard before running the command.
        climbStartPoseChooser.setDefaultOption("Blue Left", ClimbConstants.kBlueLeftStartPose);
        climbStartPoseChooser.addOption("Blue Right", ClimbConstants.kBlueRightStartPose);
        climbStartPoseChooser.addOption("Red Left", ClimbConstants.kRedLeftStartPose);
        climbStartPoseChooser.addOption("Red Right", ClimbConstants.kRedRightStartPose);

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

    private void configureDashboardCommands() {
        // putData publishes a clickable command button to SmartDashboard.
        SmartDashboard.putData("Climb Command", new ClimbCommand(robotDrive, climber, this::getSelectedClimbStartPose));
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
    }

    private Pose2d getSelectedClimbStartPose() {
        Pose2d selectedPose = climbStartPoseChooser.getSelected();
        return selectedPose != null ? selectedPose : ClimbConstants.kBlueLeftStartPose;
    }

    private void configureVisionTuning() {
        SmartDashboard.putData("VisionTuningPath", new VisionTuningPath(robotDrive));
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

        Trigger pointRearToHub, intakeToggle, zeroHeading, extendToggle, climbTrigger, shootTrigger, turboSpeedTrigger;
        Trigger killshooter, killIntake, climberExtend, climberRetract, vortexSpeedShot, jiggleIntake, calibrateIntake;
        DoubleSupplier leftY, leftX;

        if (driverPS5 != null) {
            pointRearToHub = driverPS5.square();
            intakeToggle = driverPS5.circle();
            zeroHeading = driverPS5.triangle();
            extendToggle = driverPS5.cross();
            climbTrigger = driverPS5.L1();
            turboSpeedTrigger = driverPS5.R1();
            shootTrigger = driverPS5.R2().or(driverPS5.L2());

            leftY = () -> -driverPS5.getLeftY();
            leftX = () -> -driverPS5.getLeftX();
        } else {
            pointRearToHub = driverXbox.x();
            intakeToggle = driverXbox.b();
            zeroHeading = driverXbox.y();
            extendToggle = driverXbox.a();
            climbTrigger = driverXbox.leftBumper();
            turboSpeedTrigger = driverXbox.rightBumper();
            shootTrigger = driverXbox.rightTrigger().or(driverXbox.leftTrigger());

            leftY = () -> -driverXbox.getLeftY();
            leftX = () -> -driverXbox.getLeftX();
        }

        if (operatorPS5 !=null) {
            killshooter = operatorPS5.square();
            killIntake = operatorPS5.circle();
            climberExtend = operatorPS5.triangle();
            climberRetract = operatorPS5.cross();
            vortexSpeedShot = operatorPS5.R2();
            jiggleIntake = operatorPS5.L2();
            calibrateIntake = operatorPS5.R1();
        } else {
            killshooter = operatorXbox.x();
            killIntake = operatorXbox.b();
            climberExtend = operatorXbox.y();
            climberRetract = operatorXbox.a();
            vortexSpeedShot = operatorXbox.rightTrigger();
            jiggleIntake = operatorXbox.leftTrigger();
            calibrateIntake = operatorXbox.rightBumper();


        }

        // ========== Vision-Based Navigation Commands ==========

        // Main controller commands

        intakeToggle.onTrue(Commands.runOnce(m_ballIntake::toggleRun, m_ballIntake));
        zeroHeading.onTrue(new ZeroHeadingCommand(robotDrive));
        extendToggle.onTrue(new ToggleBallIntakeExtendCommand(m_ballIntake));
        climbTrigger.onTrue(new ClimbCommand(robotDrive, climber, this::getSelectedClimbStartPose));

        pointRearToHub.whileTrue(new PointRearToAllianceHubCommand(robotDrive));
        shootTrigger.whileTrue(new ShootCommand(m_shooter));
        turboSpeedTrigger.whileTrue(
            Commands.startEnd(
                robotDrive::enableTurboSpeed,
                robotDrive::disableTurboSpeed,
                robotDrive
            )
        );

        // Operator controller commands
        killshooter.onTrue(Commands.runOnce(m_shooter::killShooter, m_shooter));
        killIntake.onTrue(Commands.runOnce(m_ballIntake::stopRun, m_ballIntake));
        jiggleIntake.onTrue(Commands.runOnce(m_ballIntake::JiggleIntake, m_ballIntake));
        calibrateIntake.onTrue(new CalibrateBallIntakeCommand(m_ballIntake));

        climberExtend.whileTrue(Commands.startEnd(climber::extendClimber, climber::stop, climber));
        climberRetract.whileTrue(Commands.startEnd(climber::retractClimber, climber::stop, climber));
        vortexSpeedShot.whileTrue(new VortexSpeedShotCommand(m_shooter));

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
            );
    }
}