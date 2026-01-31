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
        double ballPitch = vision.getBallPosition().pitch;
        double ballArea = vision.getBallPosition().area;
        
        // Calculate rotation speed using PID
        // PID output is based on error (ball yaw - setpoint of 0)
        double pidOutput = rotationController.calculate(ballYaw);
        double turnSpeed = pidOutput;
        
        // Limit rotation speed for safety
        double turnSpeedLimited = Math.max(-BallTrackingConstants.kMaxRotationSpeed, 
                           Math.min(BallTrackingConstants.kMaxRotationSpeed, turnSpeed));
        
        // Debug output - detailed tracking information
        System.out.printf("=== BALL TRACKING DEBUG ===\n");
        System.out.printf("Ball Yaw: %.2f° (negative=left, positive=right)\n", ballYaw);
        System.out.printf("Ball Pitch: %.2f° (negative=down, positive=up)\n", ballPitch);
        System.out.printf("Ball Area: %.2f%% of camera view\n", ballArea);
        System.out.printf("PID Setpoint: %.2f°\n", rotationController.getSetpoint());
        System.out.printf("PID Error: %.2f°\n", ballYaw - rotationController.getSetpoint());
        System.out.printf("PID Output (raw): %.4f\n", pidOutput);
        System.out.printf("Turn Speed (limited): %.4f (negative=CW/right, positive=CCW/left)\n", turnSpeedLimited);
        System.out.printf("At Setpoint: %b (tolerance: %.2f°)\n", 
                         rotationController.atSetpoint(), BallTrackingConstants.kYawTolerance);
        System.out.printf("==========================\n\n");

        // Drive with manual translation control and automatic rotation

        //TEMPORRARY DISABLED DRIVE
        drive.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), turnSpeedLimited, true, true);
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
