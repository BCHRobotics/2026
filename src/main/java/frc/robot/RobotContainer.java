package frc.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.drivetrain.FacePointCommand;
import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.drivetrain.VisionTuningPath;
import frc.robot.commands.drivetrain.ZeroHeadingCommand;
import frc.robot.commands.ballintake.CalibrateBallIntakeCommand;
import frc.robot.commands.ballintake.ToggleBallIntakeExtendCommand;
import frc.robot.commands.ballintake.ToggleBallIntakeRunCommand;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.webserver.VisionWebServer;
import frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

public class RobotContainer {
    // Subsystems
    private final Drivetrain robotDrive = new Drivetrain();
    private final BallIntake m_ballIntake = new BallIntake();

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
        configurePathPlannerPidChoosers();
        configureVisionTuning();

        // NamedCommands.registerCommand("name", getAutonomousCommand());

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

        Trigger alignToTag, intakeToggle, zeroHeading, extendToggle;
        DoubleSupplier leftY, leftX;

        if (driverPS5 != null) {
            alignToTag = driverPS5.square();
            intakeToggle = driverPS5.circle();
            zeroHeading = driverPS5.triangle();
            extendToggle = driverPS5.cross();

            leftY = () -> -driverPS5.getLeftY();
            leftX = () -> -driverPS5.getLeftX();
        } else {
            alignToTag = driverXbox.x();
            intakeToggle = driverXbox.b();
            zeroHeading = driverXbox.y();
            extendToggle = driverXbox.a();

            leftY = () -> -driverXbox.getLeftY();
            leftX = () -> -driverXbox.getLeftX();
        }

        // ========== Vision-Based Navigation Commands ==========

        // Square/X button: Navigate to 1 meter in front of an AprilTag
        alignToTag.whileTrue(
                new FacePointCommand(robotDrive, leftY, // Forward/backward (inverted)
                        leftX, 11.945, 4.029, 2) // Safety timeout
        );

        intakeToggle.onTrue(new ToggleBallIntakeRunCommand(m_ballIntake));

        // driverController.cross().whileTrue(
        //     new GoToPositionCommand(m_robotDrive,10.0, 7.0,0.0)
        //             .withTimeout(10.0) // Safety timeout
        // );

        // Reset gyro heading to zero (forward)
        zeroHeading.onTrue(new ZeroHeadingCommand(robotDrive));

        // Cross: Toggle between retracted home and configured extended position.
        extendToggle.onTrue(new ToggleBallIntakeExtendCommand(m_ballIntake));
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
                .andThen(new ConditionalCommand(
                        selectedAuto,
                        new PrintCommand("BallIntake calibration failed; skipping autonomous command."),
                        m_ballIntake::isCalibrated));

    }
}