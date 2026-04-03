package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

/**
 * Teleop drive command for swerve drivetrain.
 * Uses supplier pattern for dynamic joystick values.
 */
public class TeleopDriveCommand extends Command {
    private final Drivetrain m_drivetrain;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rotation;
    private final boolean m_fieldRelative;
    private final boolean m_rateLimit;

    /**
     * Creates a new TeleopDriveCommand.
     *
     * @param drivetrain The drivetrain subsystem
     * @param xSpeed Supplier for forward/backward speed (-1.0 to 1.0)
     * @param ySpeed Supplier for left/right speed (-1.0 to 1.0)
     * @param rotation Supplier for rotation speed (-1.0 to 1.0)
     * @param fieldRelative Whether to drive field-relative (true) or robot-relative (false)
     * @param rateLimit Whether to apply slew rate limiting for smoother acceleration
     */
    public TeleopDriveCommand(
            Drivetrain drivetrain,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotation,
            boolean fieldRelative,
            boolean rateLimit) {
        m_drivetrain = drivetrain;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotation = rotation;
        m_fieldRelative = fieldRelative;
        m_rateLimit = rateLimit;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Get joystick values with deadband
        double xSpeed = MathUtil.applyDeadband(m_xSpeed.getAsDouble(), OIConstants.kDriveDeadband);
        double ySpeed = MathUtil.applyDeadband(m_ySpeed.getAsDouble(), OIConstants.kDriveDeadband);
        double rotation = MathUtil.applyDeadband(m_rotation.getAsDouble(), OIConstants.kTurnDeadband);

        // Drive the robot
        m_drivetrain.drive(xSpeed, ySpeed, rotation, m_fieldRelative, m_rateLimit);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop instantly via setChassisSpeeds, bypassing the slew rate limiter,
        // so the robot doesn't coast after auto ends or command is interrupted
        m_drivetrain.setChassisSpeeds(new ChassisSpeeds());
    }
}
