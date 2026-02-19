package frc.robot;

import frc.robot.commands.drivetrain.FacePointCommand;
import frc.robot.commands.drivetrain.GoToPositionCommand;
import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.vision.AlignToAprilTagCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.webserver.VisionWebServer;
// import frc.robot.Constants.ActuatorConstants;  // DISABLED: Not needed when actuators disabled
// import frc.robot.Constants.Actuator2Constants;  // DISABLED: Not needed when actuators disabled
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

// import edu.wpi.first.math.MathUtil;  // DISABLED: Not needed when actuators disabled
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
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
    private final VisionWebServer webServer = new VisionWebServer(vision);

    // Controllers
    CommandPS5Controller driverController = new CommandPS5Controller(OIConstants.kMainControllerPort);
    
    // Autonomous chooser
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    private final SendableChooser<PIDConstants> m_ppTranslationPidChooser = new SendableChooser<>();
    private final SendableChooser<PIDConstants> m_ppRotationPidChooser = new SendableChooser<>();

    //The container for the robot, initializing everything and setting up the controller chooser
    public RobotContainer() {
        // Start data logging for AdvantageScope
        DataLogManager.start();
        
        // Connect Vision subsystem to Drivetrain for diagnostics
        robotDrive.setVision(vision);
        
        // Start the web server for vision diagnostics
        webServer.start();
        
        // Configure autonomous chooser with available auto paths
        configureAutonomousChooser();
        
        // Configure default commands
        configureDefaultCommands();

        // Configure PathPlanner PID choosers
        configurePathPlannerPidChoosers();
        
        // Configure button bindings
        configureBindings();
    }
    
    /**
     * Configures the autonomous command chooser with all available auto paths.
     * Autos are loaded from the deploy/pathplanner/autos directory.
     * The chooser is displayed on SmartDashboard for driver station selection.
     */
    private void configureAutonomousChooser() {
        // Add autonomous options
        // Note: PathPlanner autos require AutoBuilder to be configured in the drivetrain
        m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
        
        m_autoChooser.addOption("Auto Right", new PathPlannerAuto("Auto Right"));
        m_autoChooser.addOption("Auto Climber", new PathPlannerAuto("Auto Climber"));
        m_autoChooser.addOption("Test Climber Left", new PathPlannerAuto("Test Climber Left"));
        m_autoChooser.addOption("Test Climber Centre", new PathPlannerAuto("Test Climber Centre"));  
        
        // Put the chooser on SmartDashboard for driver selection
        SmartDashboard.putData("Auto Mode", m_autoChooser);
    }

    /**
     * Configures Shuffleboard choosers for PathPlanner translation/rotation PID presets.
     */
    private void configurePathPlannerPidChoosers() {
        m_ppTranslationPidChooser.setDefaultOption(
            "Default (2.0, 1.0, 0.0)",
            AutoConstants.translationConstants
        );
        m_ppTranslationPidChooser.addOption("Soft (1.2, 0.0, 0.0)", new PIDConstants(1.2, 0.0, 0.0));
        m_ppTranslationPidChooser.addOption("Aggressive (3.0, 1.2, 0.0)", new PIDConstants(3.0, 1.2, 0.0));

        m_ppRotationPidChooser.setDefaultOption(
            "Default (1.0, 0.0, 0.0)",
            AutoConstants.rotationConstants
        );
        m_ppRotationPidChooser.addOption("Soft (0.6, 0.0, 0.0)", new PIDConstants(0.6, 0.0, 0.0));
        m_ppRotationPidChooser.addOption("Aggressive (1.6, 0.0, 0.0)", new PIDConstants(1.6, 0.0, 0.0));

        SmartDashboard.putData("PP Translation PID", m_ppTranslationPidChooser);
        SmartDashboard.putData("PP Rotation PID", m_ppRotationPidChooser);
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
     * Returns the command to run during autonomous period.
     * 
     * Selects the autonomous command from the SmartDashboard chooser.
     * Defaults to "Auto 1 Left" if no selection is made.
     * 
     * @return the autonomous command selected from dashboard
     */
    public Command getAutonomousCommand() {
        PIDConstants translationConstants = m_ppTranslationPidChooser.getSelected();
        PIDConstants rotationConstants = m_ppRotationPidChooser.getSelected();
        if (translationConstants == null) {
            translationConstants = AutoConstants.translationConstants;
        }
        if (rotationConstants == null) {
            rotationConstants = AutoConstants.rotationConstants;
        }
        robotDrive.configureAutoBuilder(translationConstants, rotationConstants);

        // return new PathPlannerAuto("Test 4 Auto");
        return m_autoChooser.getSelected();
    
    }
}
