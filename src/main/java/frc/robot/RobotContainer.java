package frc.robot;

import frc.robot.commands.drivetrain.TeleopDriveCommand;
import frc.robot.commands.vision.AlignToAprilTagCommand;
import frc.robot.commands.vision.GoToAprilTagCommand;
// import frc.robot.subsystems.Actuator;  // DISABLED: Example subsystem - hardware does not exist
// import frc.robot.subsystems.Actuator2;  // DISABLED: Example subsystem - hardware does not exist
// import frc.robot.subsystems.BallIntake;  // DISABLED: Example subsystem - hardware does not exist
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
// import frc.robot.Constants.ActuatorConstants;  // DISABLED: Not needed when actuators disabled
// import frc.robot.Constants.Actuator2Constants;  // DISABLED: Not needed when actuators disabled
import frc.robot.Constants.OIConstants;

import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.math.MathUtil;  // DISABLED: Not needed when actuators disabled
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
    // Subsystems
    private final Drivetrain m_robotDrive = new Drivetrain();
    
    /**
     * Vision subsystem for AprilTag detection and pose estimation.
     * 
     * Integrates PhotonVision with the drivetrain to provide vision-based
     * localization using the 2026 Rebuilt AprilTag field layout.
     * 
     * IMPORTANT: Requires PhotonVision to be running on a coprocessor and
     * camera configuration to be completed in VisionConstants.
     */
    private final Vision m_vision = new Vision(m_robotDrive);
    
    /* DISABLED: Example subsystems - Hardware does not physically exist on this robot
     * Kept in code as examples for future development
     * 
    /**
     * PID-controlled actuator subsystem (using SPARK MAX onboard PID).
     * 
     * Provides closed-loop position control for mechanisms like elevators,
     * arms, or intakes using a NEO motor with SPARK MAX controller.
     */
    // private final Actuator m_actuator = new Actuator();
    
    /**
     * WPILib PID-controlled actuator subsystem (using RoboRIO-based PID).
     * 
     * Similar to Actuator but uses WPILib's PIDController for more flexibility
     * and easier real-time tuning via SmartDashboard.
     */
    // private final Actuator2 m_actuator2 = new Actuator2();
    
    /**
     * Ball Intake subsystem for collecting and controlling game pieces.
     * 
     * Variable speed motor for intaking, ejecting, and holding balls.
     * Controlled via button inputs for different operating modes.
     */
    // private final BallIntake m_ballIntake = new BallIntake();
    // */

    // Controllers
    CommandPS5Controller driverController = new CommandPS5Controller(OIConstants.kMainControllerPort);
    
    // Autonomous chooser
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {
        // Connect Vision subsystem to Drivetrain for diagnostics
        m_robotDrive.setVision(m_vision);
        
        // Configure autonomous chooser with available auto paths
        configureAutonomousChooser();
        
        // Configure default commands
        configureDefaultCommands();
        
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
        m_autoChooser.addOption("Auto 1 Left", new PathPlannerAuto("Auto 1 Left"));
        m_autoChooser.addOption("Auto Right", new PathPlannerAuto("Auto Right"));
        m_autoChooser.addOption("Auto Climber", new PathPlannerAuto("Auto Climber"));
        
        // Put the chooser on SmartDashboard for driver selection
        SmartDashboard.putData("Auto Mode", m_autoChooser);
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
                () -> -driverController.getLeftY(),    // Forward/backward (inverted)
                () -> -driverController.getLeftX(),    // Left/right (inverted)
                () -> -driverController.getRightX(),   // Rotation (inverted)
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
     * PS5 Controller Layout (Active):
     * - Left Stick: Drive control
     * - Square: Navigate to AprilTag 12 (autonomous positioning)
     * - Options: Vision alignment to AprilTag 4
     * 
     * PS5 Controller Layout (Disabled - Example Code):
     * - Right Stick: Actuator 1 manual control (DISABLED)
     * - Cross/Circle/Triangle: Actuator 1 preset positions (DISABLED)
     * - D-Pad: Actuator 2 preset positions (DISABLED)
     * - L1/R1: Ball Intake control (DISABLED)
     * - L2/R2: Actuator 2 manual control (DISABLED)
     * - Create: Reset actuator encoder (DISABLED)
     */
    private void configureBindings() {
        /* DISABLED: Actuator 1 and Actuator 2 controls - Hardware does not exist
         * Kept as example code for future development
         * 
        // ========== Actuator 1 Default Command: Manual Control ==========
        // Right stick Y-axis controls actuator with deadband and scaling
        m_actuator.setDefaultCommand(
            Commands.run(
                () -> {
                    double speed = -driverController.getRightY(); // Invert Y axis (up = positive)
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
        
        // Cross button (PS5): Move to home position (retracted)
        driverController.cross().onTrue(
            Commands.runOnce(() -> m_actuator.setPosition(ActuatorConstants.kHomePosition), m_actuator)
                    .withName("Actuator Home")
        );
        
        // Circle button (PS5): Move to mid position
        driverController.circle().onTrue(
            Commands.runOnce(() -> m_actuator.setPosition(ActuatorConstants.kMidPosition), m_actuator)
                    .withName("Actuator Mid")
        );
        
        // Triangle button (PS5): Move to max extension position
        driverController.triangle().onTrue(
            Commands.runOnce(() -> m_actuator.setPosition(ActuatorConstants.kMaxPosition), m_actuator)
                    .withName("Actuator Max")
        );
        */
        
        // ========== Vision-Based Navigation Commands ==========
        
        // Square button (PS5): Navigate to 1 meter in front of AprilTag 28
        // Uses vision-based autonomous navigation to position the robot
        // Runs while button is held, cancels when released
        driverController.square().whileTrue(
            new GoToAprilTagCommand(m_vision, m_robotDrive, 28, 1.0)
                    .withTimeout(10.0) // Safety timeout
        );

        // Reset gyro heading to zero (forward)
        driverController.triangle().onTrue(
            Commands.runOnce(() -> m_robotDrive.zeroHeading())
        );
        
        /* DISABLED: Actuator 2 controls - Hardware does not exist
         * 
        // ========== Actuator 2 Manual Control ==========
        // L2 and R2 triggers for manual control
        m_actuator2.setDefaultCommand(
            Commands.run(
                () -> {
                    double speed = 0;
                    
                    // R2 trigger = extend
                    if (driverController.R2().getAsBoolean()) {
                        speed = Actuator2Constants.kManualSpeedScale;
                    }
                    // L2 trigger = retract
                    else if (driverController.L2().getAsBoolean()) {
                        speed = -Actuator2Constants.kManualSpeedScale;
                    }
                    
                    m_actuator2.setManualSpeed(speed);
                },
                m_actuator2
            ).withName("Manual Actuator2 Control")
        );
        
        // ========== Actuator 2 Preset Position Buttons (D-pad) ==========
        
        // D-pad Down: Move to home position (retracted)
        driverController.povDown().onTrue(
            Commands.runOnce(() -> m_actuator2.setPosition(Actuator2Constants.kHomePosition), m_actuator2)
                    .withName("Actuator2 Home")
        );
        
        // D-pad Left/Right: Move to mid position
        driverController.povLeft().or(driverController.povRight()).onTrue(
            Commands.runOnce(() -> m_actuator2.setPosition(Actuator2Constants.kMidPosition), m_actuator2)
                    .withName("Actuator2 Mid")
        );
        
        // D-pad Up: Move to max extension position
        driverController.povUp().onTrue(
            Commands.runOnce(() -> m_actuator2.setPosition(Actuator2Constants.kMaxPosition), m_actuator2)
                    .withName("Actuator2 Max")
        );
        
        // Create button (PS5): Reset encoder to zero (run when at known home position)
        driverController.create().onTrue(
            Commands.runOnce(() -> m_actuator2.resetPosition(), m_actuator2)
                    .withName("Reset Actuator2 Encoder")
        );
        */
        
        // ========== Vision Alignment Commands ==========
        // Options button (PS5): Align to nearest AprilTag for autonomous scoring
        // NOTE: This is an example - tune PID values in AlignToAprilTagCommand for your robot
        driverController.options().whileTrue(
            new AlignToAprilTagCommand(m_vision, m_robotDrive, 4)
                    .withTimeout(5.0) // Safety timeout
        );
        
        // NOTE: You can add more vision alignment commands here:
        // - Align to different tags for different scoring positions
        // - Combine with actuator commands for full scoring sequences
        // Example:
        // driverController.touchpad().whileTrue(
        //     new AlignToAprilTagCommand(m_vision, m_robotDrive, 7)
        // );
        
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
                    return new PathPlannerAuto("Test 4 Auto");
        // Command selectedAuto = m_autoChooser.getSelected();
        // System.out.println("Test run");
        // // Return selected auto, or default to Auto 1 Left if none selected
        // System.out.println(selectedAuto.getName());
        // if (selectedAuto != null) {
        //     return selectedAuto;
        // } else {
        //    System.out.println("we are getting into this path code");
        //     return new PathPlannerAuto("Test Path");
        // }
    }
}
