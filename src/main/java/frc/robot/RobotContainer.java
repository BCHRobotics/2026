package frc.robot;

import frc.robot.commands.drivetrain.FacePointCommand;
import frc.robot.commands.drivetrain.GoToPositionCommand;
import frc.robot.commands.drivetrain.GoToPositionRelativeCommand;
import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.vision.AlignToAprilTagCommand;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

public class RobotContainer {
    // Subsystems
    private final Drivetrain robotDrive = new Drivetrain();
    
    //Vision subsystem for AprilTag detection and pose estimation.
    private final Vision vision = new Vision(robotDrive);
    
    // Web server for vision diagnostics running on RoboRIO port 8082.
    private final VisionWebServer webServer = new VisionWebServer(vision);

    // Controllers
    CommandPS5Controller driverPS5;
    CommandXboxController driverXbox;
    
    // Autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Field pose chooser — selects where the robot drives when the pose-nav button is held
    private final SendableChooser<Pose2d> fieldPoseChooser = new SendableChooser<>();
    private final SendableChooser<PIDConstants> ppTranslationPidChooser = new SendableChooser<>();
    private final SendableChooser<PIDConstants> ppRotationPidChooser = new SendableChooser<>();

    //The container for the robot, initializing everything and setting up the controller chooser
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

        //NamedCommands.registerCommand("name", getAutonomousCommand());
        
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
     *   red_x = fieldLength - x,  red_y = fieldWidth - y,  red_heading = 180 - heading
     */
    private void configureFieldPoseChooser() {
        // Climber positions (final waypoints from PathPlanner paths, blue alliance)
        fieldPoseChooser.setDefaultOption("Climber 1", new Pose2d(1.037, 2.800, Rotation2d.fromDegrees(-90.0)));
        fieldPoseChooser.addOption(      "Climber 2", new Pose2d(1.061, 4.600, Rotation2d.fromDegrees( 90.0)));
        fieldPoseChooser.addOption(      "Climber 3", new Pose2d(1.037, 2.800, Rotation2d.fromDegrees(-90.0)));
        fieldPoseChooser.addOption(      "Climber 4", new Pose2d(1.061, 4.600, Rotation2d.fromDegrees( 90.0)));

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

        SmartDashboard.putData("PP Translation PID", ppTranslationPidChooser);
        SmartDashboard.putData("PP Rotation PID", ppRotationPidChooser);
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
                leftY,    // Forward/backward (inverted)
                leftX,    // Left/right (inverted)
                rightX,   // Rotation (inverted)
                OIConstants.kFieldRelative,            // Field-relative driving
                OIConstants.kRateLimited               // Enable slew rate limiting
            )
        );
        
        // CRITICAL: Set max speed for drivetrain (required for movement)
        robotDrive.setSpeedPercent();
    }
    
    // Configures button and trigger bindings for controllers.
    private void configureBindings() {

        Trigger alignToTag, goToPosition, zeroHeading, autoScoring, goToSelectedPose;
        DoubleSupplier leftY, leftX;

        if (driverPS5 != null) {
            alignToTag = driverPS5.square();
            goToPosition = driverPS5.circle();
            zeroHeading = driverPS5.triangle();
            autoScoring = driverPS5.options();
            goToSelectedPose = driverPS5.cross();
            
            leftY = () -> -driverPS5.getLeftY();
            leftX = () -> -driverPS5.getLeftX();
        } else {
            alignToTag = driverXbox.x();
            goToPosition = driverXbox.b();
            zeroHeading = driverXbox.y();
            autoScoring = driverXbox.start(); // Using Start button like Options
            goToSelectedPose = driverXbox.a();

            leftY = () -> -driverXbox.getLeftY();
            leftX = () -> -driverXbox.getLeftX();
        }
        
        // ========== Vision-Based Navigation Commands ==========
        
        // Square/X button: Navigate to 1 meter in front of an AprilTag
        alignToTag.whileTrue(
            new FacePointCommand(robotDrive, leftY,    // Forward/backward (inverted)
                leftX, 11.945, 4.029, 2) // Safety timeout
        );

         goToPosition.whileTrue(
             new GoToPositionCommand(robotDrive,10.0, 4.0,0.0)
                     .withTimeout(10.0) // Safety timeout
         );
 
        // driverController.cross().whileTrue(
        //     new GoToPositionCommand(m_robotDrive,10.0, 7.0,0.0)
        //             .withTimeout(10.0) // Safety timeout
        // );

        // Reset gyro heading to zero (forward)
        zeroHeading.onTrue(
            Commands.runOnce(() -> robotDrive.zeroHeading())
        );

        // Cross/A: Drive to the pose selected in the SmartDashboard "Field Pose" chooser.
        // Uses alliance-relative coordinates — automatically mirrors for red alliance.
        goToSelectedPose.whileTrue(
            Commands.defer(() -> {
                Pose2d pose = fieldPoseChooser.getSelected();
                if (pose == null) return Commands.none();
                return new GoToPositionRelativeCommand(
                    robotDrive, pose.getX(), pose.getY(), pose.getRotation().getDegrees()
                ).withTimeout(10.0);
            }, Set.of(robotDrive))
        );

        // driverController.square().whileTrue(
        //     new PointToBallCommand(m_robotDrive, m_vision, () -> driverController.getLeftX(), () -> driverController.getLeftY())
        // );
        
        // ========== Vision Alignment Commands ==========
        // Options/Start button: Align to nearest AprilTag for autonomous scoring
        autoScoring.whileTrue(
            new AlignToAprilTagCommand(vision, robotDrive, 4)
                    .withTimeout(5.0) // Safety timeout
        );
        
        /* DISABLED: Ball Intake Control Buttons (Motor CAN ID 22 does not physically exist)
        // ========== Ball Intake Control Buttons ==========
        // L1 button: Intake balls (while held)
        driverController.L1().whileTrue(
            new frc.robot.commands.ballintake.IntakeCommand(m_ballIntake)
        );
        
        // R1 button: Eject balls (while held)
        driverController.R1().whileTrue(
            new frc.robot.commands.ballintake.EjectCommand(m_ballIntake)
        );
        
        // Touchpad button: Hold balls (while held) - prevents balls from falling out
        driverController.touchpad().whileTrue(
            new frc.robot.commands.ballintake.HoldCommand(m_ballIntake)
        );
        
        // PS button: Stop intake (momentary) - emergency stop
        driverController.PS().onTrue(
            new frc.robot.commands.ballintake.StopCommand(m_ballIntake)
        );
        */
    }
    
    
    //   Configures the autonomous command chooser.
      
    //   Creates a dashboard selector for autonomous modes including:
    //   - Do Nothing (safe default)
    //   - Drive Forward (simple mobility)
    //   - Align to AprilTag 0 (vision-based positioning)
     
    private void configureAutoChooser() {
        // Default option: Do nothing (safe for testing)
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        
        // Simple autonomous: Drive forward 2 meters for mobility points
        autoChooser.addOption("Drive Forward 2m",
            Commands.sequence(
                // Reset odometry to start at origin
                Commands.runOnce(() -> robotDrive.resetOdometry(new Pose2d()), robotDrive),
                
                // Drive forward at 30% speed until 2 meters traveled
                Commands.run(
                    () -> robotDrive.drive(0.3, 0, 0, false, false),
                    robotDrive
                ).until(() -> robotDrive.getPose().getX() > 2.0),
                
                // Stop driving
                Commands.runOnce(() -> robotDrive.drive(0, 0, 0, false, false), robotDrive)
            ).withName("Drive Forward Auto")
        );
        
        // Vision-based autonomous: Align to AprilTag 0
        autoChooser.addOption("Align to AprilTag 0",
            Commands.sequence(
                // Reset odometry
                Commands.runOnce(() -> robotDrive.resetOdometry(new Pose2d()), robotDrive),
                
                // Wait briefly for vision to initialize
                Commands.waitSeconds(0.5),
                
                // Use vision to align to AprilTag 0 (first tag on field)
                new AlignToAprilTagCommand(vision, robotDrive, 1)
                    .withTimeout(10.0), // 10 second timeout for safety
                
                // Stop when finished
                Commands.runOnce(() -> robotDrive.drive(0, 0, 0, false, false), robotDrive)
            ).withName("Vision Align Auto")
        );
        
        // Put chooser on SmartDashboard for driver station selection
        SmartDashboard.putData("Autonomous Mode", autoChooser);
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

        // return new PathPlannerAuto("Test 4 Auto");
        return autoChooser.getSelected();
    
    }
}