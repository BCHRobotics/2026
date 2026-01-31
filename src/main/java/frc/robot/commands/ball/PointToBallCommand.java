package frc.robot.commands.ball;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BallTrackingConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 * Command to align robot rotation to point at a detected ball using PID control.
 * 
 * Uses vision-based yaw angle feedback and PID control for smooth, accurate alignment.
 * Driver can still control translation (X/Y movement) while the robot automatically
 * rotates to face the ball.
 */
public class PointToBallCommand extends Command {
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final Vision vision;
    private final Drivetrain drive;
    
    // PID controller for rotation alignment
    private final PIDController rotationController;
    
    public PointToBallCommand(
        Drivetrain drive,
        Vision vision,
        DoubleSupplier ySpeed,
        DoubleSupplier xSpeed) {
        this.vision = vision;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.drive = drive;

        // Initialize PID controller with constants
        this.rotationController = new PIDController(
            BallTrackingConstants.kRotationP,
            BallTrackingConstants.kRotationI,
            BallTrackingConstants.kRotationD
        );
        
        // Set PID controller parameters
        rotationController.setSetpoint(0.0); // Target is to center ball (yaw = 0)
        rotationController.setTolerance(BallTrackingConstants.kYawTolerance);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Check if ball is detected
        if (vision.getBallPosition() == null) {
            // No ball detected - allow manual control only
            drive.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), 0, true, true);
            return;
        }

        // Get ball yaw angle (negative = left, positive = right)
        double ballYaw = vision.getBallPosition().yaw;
        
        // Calculate rotation speed using PID
        // PID output is based on error (ball yaw - setpoint of 0)
        double turnSpeed = rotationController.calculate(ballYaw);
        
        // Limit rotation speed for safety
        turnSpeed = Math.max(-BallTrackingConstants.kMaxRotationSpeed, 
                           Math.min(BallTrackingConstants.kMaxRotationSpeed, turnSpeed));
        
        // Optional: Debug output
        // System.out.printf("Ball Yaw: %.2f°, Turn Speed: %.3f, At Setpoint: %b\n", 
        //                   ballYaw, turnSpeed, rotationController.atSetpoint());

        // Drive with manual translation control and automatic rotation
        drive.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), turnSpeed, true, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop driving when command ends
        drive.drive(0, 0, 0, false, false);
    }
    
    /**
     * Check if robot is aligned with the ball.
     * 
     * @return true if ball yaw is within tolerance
     */
    public boolean isAligned() {
        return rotationController.atSetpoint();
    }
}
