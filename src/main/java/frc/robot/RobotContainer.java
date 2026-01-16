package frc.robot;

import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.vision.AlignToAprilTagCommand;
import frc.robot.subsystems.Actuator;
import frc.robot.subsystems.Actuator2;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.ActuatorConstants;
import frc.robot.Constants.Actuator2Constants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Subsystems
    private final Drivetrain m_robotDrive = new Drivetrain();
    
    /**
     * Vision subsystem for AprilTag detection and pose estimation.
     * 
     * Integrates PhotonVision with the drivetrain to provide vision-based
     * localization using the 2025 Reefscape AprilTag field layout.
     * 
     * IMPORTANT: Requires PhotonVision to be running on a coprocessor and
     * camera configuration to be completed in VisionConstants.
     */
    private final Vision m_vision = new Vision(m_robotDrive);
    
    /**
     * PID-controlled actuator subsystem (using SPARK MAX onboard PID).
     * 
     * Provides closed-loop position control for mechanisms like elevators,
     * arms, or intakes using a NEO motor with SPARK MAX controller.
     */
    private final Actuator m_actuator = new Actuator();
    
    /**
     * WPILib PID-controlled actuator subsystem (using RoboRIO-based PID).
     * 
     * Similar to Actuator but uses WPILib's PIDController for more flexibility
     * and easier real-time tuning via SmartDashboard.
     */
    private final Actuator2 m_actuator2 = new Actuator2();
    
    /**
     * Ball Intake subsystem for collecting and controlling game pieces.
     * 
     * Variable speed motor for intaking, ejecting, and holding balls.
     * Controlled via button inputs for different operating modes.
     */
    private final BallIntake m_ballIntake = new BallIntake();

    // Controllers
    CommandXboxController driverController_XBOX = new CommandXboxController(OIConstants.kMainControllerPort);
    
    // Autonomous chooser
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
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
        m_robotDrive.setDefaultCommand(
            new TeleopDriveCommand(
                m_robotDrive,
                () -> -driverController_XBOX.getLeftY(),    // Forward/backward (inverted)
                () -> -driverController_XBOX.getLeftX(),    // Left/right (inverted)
                () -> -driverController_XBOX.getRightX(),   // Rotation (inverted)
                OIConstants.kFieldRelative,                 // Field-relative driving
                OIConstants.kRateLimited                    // Enable slew rate limiting
            )
        );
        
        // CRITICAL: Set max speed for drivetrain (required for movement)
        m_robotDrive.setSpeedPercent();
    }
    
    /**
     * Configures button and trigger bindings for controllers.
     * 
     * Sets up:
     * - Default commands for continuous control
     * - Button bindings for preset positions
     * - Manual override controls
     * 
     * Actuator 1: Right stick Y-axis, A/B/Y/X buttons
     * Actuator 2: D-pad for preset positions, Left/Right bumpers for manual
     */
    private void configureBindings() {
        // ========== Actuator 1 Default Command: Manual Control ==========
        // Right stick Y-axis controls actuator with deadband and scaling
        m_actuator.setDefaultCommand(
            Commands.run(
                () -> {
                    double speed = -driverController_XBOX.getRightY(); // Invert Y axis (up = positive)
                    // Apply deadband to prevent drift
                    speed = MathUtil.applyDeadband(speed, ActuatorConstants.kManualDeadband);
                    // Scale speed for finer control
                    speed *= ActuatorConstants.kManualSpeedScale;
                    // Send to actuator
                    m_actuator.setManualSpeed(speed);
                },
                m_actuator
            ).withName("Manual Actuator Control")
        );
        
        // ========== Actuator 1 Preset Position Buttons ==========
        
        // A button: Move to home position (retracted)
        driverController_XBOX.a().onTrue(
            Commands.runOnce(() -> m_actuator.setPosition(ActuatorConstants.kHomePosition), m_actuator)
                    .withName("Actuator Home")
        );
        
        // B button: Move to mid position
        driverController_XBOX.b().onTrue(
            Commands.runOnce(() -> m_actuator.setPosition(ActuatorConstants.kMidPosition), m_actuator)
                    .withName("Actuator Mid")
        );
        
        // Y button: Move to max extension position
        driverController_XBOX.y().onTrue(
            Commands.runOnce(() -> m_actuator.setPosition(ActuatorConstants.kMaxPosition), m_actuator)
                    .withName("Actuator Max")
        );
        
        // X button: Reset encoder to zero (run when at known home position)
        driverController_XBOX.x().onTrue(
            Commands.runOnce(() -> m_actuator.resetPosition(), m_actuator)
                    .withName("Reset Actuator Encoder")
        );
        
        // ========== Actuator 2 Manual Control ==========
        // Left and Right bumpers for manual control
        m_actuator2.setDefaultCommand(
            Commands.run(
                () -> {
                    double speed = 0;
                    
                    // Right bumper = extend
                    if (driverController_XBOX.rightBumper().getAsBoolean()) {
                        speed = Actuator2Constants.kManualSpeedScale;
                    }
                    // Left bumper = retract
                    else if (driverController_XBOX.leftBumper().getAsBoolean()) {
                        speed = -Actuator2Constants.kManualSpeedScale;
                    }
                    
                    m_actuator2.setManualSpeed(speed);
                },
                m_actuator2
            ).withName("Manual Actuator2 Control")
        );
        
        // ========== Actuator 2 Preset Position Buttons (D-pad) ==========
        
        // D-pad Down: Move to home position (retracted)
        driverController_XBOX.povDown().onTrue(
            Commands.runOnce(() -> m_actuator2.setPosition(Actuator2Constants.kHomePosition), m_actuator2)
                    .withName("Actuator2 Home")
        );
        
        // D-pad Left/Right: Move to mid position
        driverController_XBOX.povLeft().or(driverController_XBOX.povRight()).onTrue(
            Commands.runOnce(() -> m_actuator2.setPosition(Actuator2Constants.kMidPosition), m_actuator2)
                    .withName("Actuator2 Mid")
        );
        
        // D-pad Up: Move to max extension position
        driverController_XBOX.povUp().onTrue(
            Commands.runOnce(() -> m_actuator2.setPosition(Actuator2Constants.kMaxPosition), m_actuator2)
                    .withName("Actuator2 Max")
        );
        
        // Back button: Reset encoder to zero (run when at known home position)
        driverController_XBOX.back().onTrue(
            Commands.runOnce(() -> m_actuator2.resetPosition(), m_actuator2)
                    .withName("Reset Actuator2 Encoder")
        );
        
        // ========== Vision Alignment Commands ==========
        // Start button: Align to nearest AprilTag for autonomous scoring
        // NOTE: This is an example - tune PID values in AlignToAprilTagCommand for your robot
        driverController_XBOX.start().whileTrue(
            new AlignToAprilTagCommand(m_vision, m_robotDrive, 4)
                    .withTimeout(5.0) // Safety timeout
        );
        
        // NOTE: You can add more vision alignment commands here:
        // - Align to different tags for different scoring positions
        // - Combine with actuator commands for full scoring sequences
        // Example:
        // driverController_XBOX.leftTrigger().whileTrue(
        //     new AlignToAprilTagCommand(m_vision, m_robotDrive, 7)
        // );
        
        // ========== Ball Intake Control Buttons ==========
        // Left Bumper: Intake balls (while held)
        driverController_XBOX.leftBumper().whileTrue(
            new frc.robot.commands.ballintake.IntakeCommand(m_ballIntake)
        );
        
        // Right Bumper: Eject balls (while held)
        driverController_XBOX.rightBumper().whileTrue(
            new frc.robot.commands.ballintake.EjectCommand(m_ballIntake)
        );
        
        // Left Trigger: Hold balls (while held) - prevents balls from falling out
        driverController_XBOX.leftTrigger().whileTrue(
            new frc.robot.commands.ballintake.HoldCommand(m_ballIntake)
        );
        
        // Right Trigger: Stop intake (momentary)
        driverController_XBOX.rightTrigger().onTrue(
            new frc.robot.commands.ballintake.StopCommand(m_ballIntake)
        );
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
        m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
        
        // Simple autonomous: Drive forward 2 meters for mobility points
        m_autoChooser.addOption("Drive Forward 2m",
            Commands.sequence(
                // Reset odometry to start at origin
                Commands.runOnce(() -> m_robotDrive.resetOdometry(new Pose2d()), m_robotDrive),
                
                // Drive forward at 30% speed until 2 meters traveled
                Commands.run(
                    () -> m_robotDrive.drive(0.3, 0, 0, false, false),
                    m_robotDrive
                ).until(() -> m_robotDrive.getPose().getX() > 2.0),
                
                // Stop driving
                Commands.runOnce(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            ).withName("Drive Forward Auto")
        );
        
        // Vision-based autonomous: Align to AprilTag 0
        m_autoChooser.addOption("Align to AprilTag 0",
            Commands.sequence(
                // Reset odometry
                Commands.runOnce(() -> m_robotDrive.resetOdometry(new Pose2d()), m_robotDrive),
                
                // Wait briefly for vision to initialize
                Commands.waitSeconds(0.5),
                
                // Use vision to align to AprilTag 0 (first tag on field)
                new AlignToAprilTagCommand(m_vision, m_robotDrive, 1)
                    .withTimeout(10.0), // 10 second timeout for safety
                
                // Stop when finished
                Commands.runOnce(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
            ).withName("Vision Align Auto")
        );
        
        // TODO: Add PathPlanner autonomous routines
        // Example:
        // m_autoChooser.addOption("Center 2 Coral",
        //     AutoBuilder.buildAuto("[C] 2 Coral")
        // );
        
        // Put chooser on SmartDashboard for driver station selection
        SmartDashboard.putData("Autonomous Mode", m_autoChooser);
    }

    /**
     * Returns the command to run during autonomous period.
     * 
     * @return the autonomous command selected from dashboard
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}

