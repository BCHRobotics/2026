package frc.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.drivetrain.FacePointCommand;
import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.drivetrain.VisionTuningPath;
import frc.robot.commands.drivetrain.ZeroHeadingCommand;
import frc.robot.commands.drivetrain.GoToPositionCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.ballintake.CalibrateBallIntakeCommand;
import frc.robot.commands.ballintake.ToggleBallIntakeExtendCommand;
import frc.robot.commands.ballintake.ToggleBallIntakeRunCommand;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.webserver.VisionWebServer;
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

    // Web server for vision diagnostics running on RoboRIO port 8082.
    private final VisionWebServer webServer = new VisionWebServer(vision);

    // Controllers
    CommandPS5Controller driverPS5;
    CommandXboxController driverXbox;
    // Operator controller (port 1) — always PS5, used for intake controls
    CommandPS5Controller operatorPS5 = new CommandPS5Controller(OIConstants.kBackupControllerPort);

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

        // Start data logging for AdvantageScope
        DataLogManager.start();

        // Connect Vision subsystem to Drivetrain for diagnostics
        robotDrive.setVision(vision);

        // Start the web server for vision diagnostics
        webServer.start();

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

        autoChooser.addOption("Square Auto", new PathPlannerAuto("Square Auto"));
        autoChooser.addOption("Tuning_auto", new PathPlannerAuto("Tuning_auto"));
        autoChooser.addOption("Circle Auto", new PathPlannerAuto("Circle Auto"));
        autoChooser.addOption("Test Climber Centre", new PathPlannerAuto("Test Climber Centre"));
        autoChooser.addOption("Climber-1_Auto", new PathPlannerAuto("Climber-1_Auto"));
        autoChooser.addOption("Climber-2_Auto", new PathPlannerAuto("Climber-2_Auto"));
        autoChooser.addOption("Climber-3_Auto", new PathPlannerAuto("Climber-3_Auto"));
        autoChooser.addOption("Climber-4_Auto", new PathPlannerAuto("Climber-4_Auto"));
        autoChooser.addOption("Climber-5_Auto", new PathPlannerAuto("Climber-5_Auto"));
        autoChooser.addOption("Climber-6_Auto", new PathPlannerAuto("Climber-6_Auto"));
        autoChooser.addOption("N_Climber-7_Auto", new PathPlannerAuto("N_Climber-7_Auto"));
        autoChooser.addOption("Climber-8_Auto", new PathPlannerAuto("Climber-8_Auto"));
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
        // test locations and select them from SmartDashboard before running the command.
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
                "RunClimb",
                new ClimbCommand(robotDrive, climber, this::getSelectedClimbStartPose));
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
            leftY = () -> -driverPS5.getLeftY();
            leftX = () -> -driverPS5.getLeftX();
            rightX = () -> -driverPS5.getRightX();
        } else {
            leftY = () -> -driverXbox.getLeftY();
            leftX = () -> -driverXbox.getLeftX();
            rightX = () -> -driverXbox.getRightX();
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

        Trigger alignToTag, intakeToggle, zeroHeading, extendToggle, climbTrigger, shootTrigger;
        Trigger killshooter, killIntake, goToClimb, climbtoggle;
        DoubleSupplier leftY, leftX;

        if (driverPS5 != null) {
            alignToTag = driverPS5.square();
            intakeToggle = driverPS5.circle();
            zeroHeading = driverPS5.triangle();
            extendToggle = driverPS5.cross();
            climbTrigger = driverPS5.L1();
            shootTrigger = driverPS5.R2().or(driverPS5.L2());

            leftY = () -> -driverPS5.getLeftY();
            leftX = () -> -driverPS5.getLeftX();
        } else {
            alignToTag = driverXbox.x();
            intakeToggle = driverXbox.b();
            zeroHeading = driverXbox.y();
            extendToggle = driverXbox.a();
            climbTrigger = driverXbox.leftBumper();
            shootTrigger = driverXbox.rightTrigger().or(driverXbox.leftTrigger());

            leftY = () -> -driverXbox.getLeftY();
            leftX = () -> -driverXbox.getLeftX();
        }

        if (operatorPS5 !=null) {
            killshooter = operatorPS5.square();
            killIntake = operatorPS5.circle();
            goToClimb = operatorPS5.triangle();
            climbtoggle = operatorPS5.cross();

            killshooter.onTrue(Commands.runOnce(m_shooter::killShooter, m_shooter));
            killIntake.onTrue(Commands.runOnce(m_ballIntake::stopRun, m_ballIntake));

            // goToClimb.onTrue(
            //     new GoToPositionCommand(robotDrive, 1.061, 4.600, 0.0)
            //             .withTimeout(10.0) // Safety timeout
            // );
            // climbtoggle.onTrue(new ToggleClimberExtendCommand(m_climber));

        }

        // ========== Vision-Based Navigation Commands ==========

        // Square/X button: Navigate to 1 meter in front of an AprilTag
        alignToTag.whileTrue(
                new FacePointCommand(robotDrive, leftY, // Forward/backward (inverted)
                        leftX, 11.945, 4.029, 2) // Safety timeout
        );

        intakeToggle.onTrue(Commands.runOnce(m_ballIntake::toggleRun, m_ballIntake));

        // driverController.cross().whileTrue(
        //     new GoToPositionCommand(m_robotDrive,10.0, 7.0,0.0)
        //             .withTimeout(10.0) // Safety timeout
        // );

        // Reset gyro heading to zero (forward)
        zeroHeading.onTrue(new ZeroHeadingCommand(robotDrive));

        // Cross: Toggle between retracted home and configured extended position.
        extendToggle.onTrue(new ToggleBallIntakeExtendCommand(m_ballIntake));

        // Left bumper: run the climb sequence using the currently selected climb start pose.
        climbTrigger.onTrue(new ClimbCommand(robotDrive, climber, this::getSelectedClimbStartPose));

        // R2 and L2: Shoot while held
        shootTrigger.whileTrue(new ShootCommand(m_shooter));
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
        return new ConditionalCommand(
                Commands.none(),
                new CalibrateBallIntakeCommand(m_ballIntake),
                m_ballIntake::isCalibrated);
    }
}