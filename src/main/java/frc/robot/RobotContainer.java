package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // drivetrain
    private final Drivetrain m_robotDrive = new Drivetrain();

    CommandXboxController driverController_XBOX = new CommandXboxController(OIConstants.kMainControllerPort);

    /**
     * The container for the robot, initializing everything and setting up the controller chooser
     */
    public RobotContainer() {

    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
