package frc.robot.commands.ball;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class PointToBallCommand extends Command {
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final Vision vision;

    private final Drivetrain drive;
    
    public PointToBallCommand(
        Drivetrain drive,
        Vision vision,
        DoubleSupplier ySpeed,
            DoubleSupplier xSpeed) {
        this.vision = vision;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (vision.getBallPosition() == null) {return;}
        double turnSpeed = -vision.getBallPosition().yaw;

        if (Math.abs(turnSpeed) < 2) {
            drive.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), 0, true, true);
        }

        //System.out.println(vision.getBallPosition().yaw);

        double clamp = 0.4;
        turnSpeed = MathUtil.clamp(turnSpeed, -clamp, clamp) * 0.5;

        // Drive the robot
        drive.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), turnSpeed, true, true);
    }
}
