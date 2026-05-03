package frc.robot.commands.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Command that uses PathPlanner's AutoBuilder.pathfindToPose() to navigate
 * the robot to a selected field position with full motion profiling.
 *
 * <p>The target pose is supplied lazily at initialize() time, so the driver
 * can change the SmartDashboard selection between button presses and always
 * get the most recently selected pose.
 *
 * <p>The command ends naturally when PathPlanner reports it has reached the
 * target, or is interrupted when the trigger button is released (whileTrue).
 */
public class PathfindToPoseCommand extends Command {

    /** PathPlanner motion constraints for pathfinding. */
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0,          // maxVelocityMps
        2.0,          // maxAccelerationMpsSq
        Math.PI,      // maxAngularVelocityRps
        Math.PI       // maxAngularAccelerationRpsSq
    );

    private final Drivetrain m_drivetrain;
    private final Supplier<Pose2d> m_poseSupplier;

    /** The inner PathPlanner command generated at initialize(). */
    private Command m_pathfindCommand;

    /**
     * Creates a new PathfindToPoseCommand.
     *
     * @param drivetrain   The drivetrain subsystem (required).
     * @param poseSupplier Supplier called at initialize() to get the target pose
     *                     (typically backed by a SmartDashboard SendableChooser).
     */
    public PathfindToPoseCommand(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
        m_drivetrain = drivetrain;
        m_poseSupplier = poseSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d targetPose = m_poseSupplier.get();
        Pose2d currentPose = m_drivetrain.getPose();

        System.out.println("PathfindToPose: Navigating to " + targetPose);
        System.out.println("PathfindToPose: Current pose   " + currentPose);

        Logger.recordOutput("PathfindToPose/TargetPose/X",        targetPose.getX());
        Logger.recordOutput("PathfindToPose/TargetPose/Y",        targetPose.getY());
        Logger.recordOutput("PathfindToPose/TargetPose/Degrees",  targetPose.getRotation().getDegrees());
        Logger.recordOutput("PathfindToPose/StartPose/X",         currentPose.getX());
        Logger.recordOutput("PathfindToPose/StartPose/Y",         currentPose.getY());
        Logger.recordOutput("PathfindToPose/StartPose/Degrees",   currentPose.getRotation().getDegrees());

        m_pathfindCommand = AutoBuilder.pathfindToPose(targetPose, CONSTRAINTS);
        m_pathfindCommand.initialize();
    }

    @Override
    public void execute() {
        if (m_pathfindCommand != null) {
            m_pathfindCommand.execute();

            Pose2d current = m_drivetrain.getPose();
            Logger.recordOutput("PathfindToPose/CurrentPose/X",      current.getX());
            Logger.recordOutput("PathfindToPose/CurrentPose/Y",      current.getY());
            Logger.recordOutput("PathfindToPose/CurrentPose/Degrees", current.getRotation().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return m_pathfindCommand != null && m_pathfindCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_pathfindCommand != null) {
            m_pathfindCommand.end(interrupted);
            m_pathfindCommand = null;
        }
        // Stop the drivetrain on release or arrival
        m_drivetrain.drive(0, 0, 0, false, false);

        if (interrupted) {
            System.out.println("PathfindToPose: Interrupted (button released or preempted)");
        } else {
            System.out.println("PathfindToPose: Arrived at target pose");
            System.out.println("PathfindToPose: Final pose " + m_drivetrain.getPose());
        }
        Logger.recordOutput("PathfindToPose/Active", false);
    }
}
