package frc.robot;

import frc.robot.commands.drivetrain.FacePointCommand;
import frc.robot.commands.drivetrain.GoToPositionCommand;
import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.vision.AlignToAprilTagCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.webserver.VisionWebServer;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
    // Subsystems
    private final Drivetrain robotDrive = new Drivetrain();
    
    //Vision subsystem for AprilTag detection and pose estimation.
    private final Vision vision = new Vision(robotDrive);
    
    // Web server for vision diagnostics running on RoboRIO port 8082.
    private final VisionWebServer webServer = new VisionWebServer(m_vision);

    // Controllers
    CommandPS5Controller driverController = new CommandPS5Controller(OIConstants.kMainControllerPort);
    
    // Autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    //The container for the robot, initializing everything and setting up the controller chooser
    public RobotContainer() {
        // Connect Vision subsystem to Drivetrain for diagnostics
        robotDrive.setVision(vision);
        
        // Start the web server for vision diagnostics
        webServer.start();
        
        // Configure default commands
        configureDefaultCommands();
        
        // Configure button bindings
        configureBindings();
        
        // Configure autonomous chooser
        configureAutoChooser();
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
        robotDrive.setDefaultCommand(
            new TeleopDriveCommand(
                robotDrive,
                () -> -driverController.getLeftY(),    // Forward/backward (inverted)
                () -> -driverController.getLeftX(),    // Left/right (inverted)
                () -> -driverController.getRightX(),   // Rotation (inverted)
                OIConstants.kFieldRelative,            // Field-relative driving
                OIConstants.kRateLimited               // Enable slew rate limiting
            )
        );
        
        // CRITICAL: Set max speed for drivetrain (required for movement)
        robotDrive.setSpeedPercent();
    }
    
    // Configures button and trigger bindings for controllers.
    private void configureBindings() {
        
        // ========== Vision-Based Navigation Commands ==========
        
        // Square button (PS5): Navigate to 1 meter in front of an AprilTag
        // Uses vision-based autonomous navigation to position the robot
        // Runs while button is held, cancels when released
        driverController.square().whileTrue(
            new FacePointCommand(robotDrive, () -> -driverController.getLeftY(),    // Forward/backward (inverted)
                () -> -driverController.getLeftX(), 11.945, 4.029, 2) // Safety timeout
        );

         driverController.circle().whileTrue(
             new GoToPositionCommand(robotDrive,10.0, 4.0,0.0)
                     .withTimeout(10.0) // Safety timeout
         );
 
        // driverController.cross().whileTrue(
        //     new GoToPositionCommand(m_robotDrive,10.0, 7.0,0.0)
        //             .withTimeout(10.0) // Safety timeout
        // );

        // Reset gyro heading to zero (forward)
        driverController.triangle().onTrue(
            Commands.runOnce(() -> robotDrive.zeroHeading())
        );

        // driverController.square().whileTrue(
        //     new PointToBallCommand(m_robotDrive, m_vision, () -> driverController.getLeftX(), () -> driverController.getLeftY())
        // );
        
        // ========== Vision Alignment Commands ==========
        // Options button (PS5): Align to nearest AprilTag for autonomous scoring
        driverController.options().whileTrue(
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
    
    /**
     * Configures the autonomous command chooser.
     * 
     * Creates a dashboard selector for autonomous modes including:
     * - Do Nothing (safe default)
     * - Drive Forward (simple mobility)
     * - Align to AprilTag 0 (vision-based positioning)
     */
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
        return autoChooser.getSelected();
    }
}
