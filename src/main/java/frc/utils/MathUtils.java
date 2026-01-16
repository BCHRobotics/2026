package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/*
 * This script is for helper functions related to vision and related math
 * There should only be public static functions in here
 */
public class MathUtils {
    /**
     * A function that, given a transform2d as a vector, rotates it by an angle
     * @param inputMatrix The input vector
     * @param angle Angle to rotate by IN RADIANS
     * @return the vector rotated by the angle
     */
    public static Translation2d applyRotationMatrix(Translation2d inputMatrix, double angle) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(angle);
        double cosHeading = Math.cos(angle);

        // Create field-relative coordinates using the heading and robot-relative coords
        double fieldX = inputMatrix.getX() * cosHeading + inputMatrix.getY() * -sinHeading;
        double fieldY = inputMatrix.getX() * sinHeading + inputMatrix.getY() * cosHeading;

        // Create the transform2d object
        Translation2d rotatedVector = new Translation2d(fieldX, fieldY);

        return rotatedVector;
    }

    public static double fixAngle(double a) {
        double remainder = a % 360;

        double clampedValue = remainder;

        if (remainder > 180) {
            clampedValue = remainder - 360;
        }
        else if (remainder < -180) {
            clampedValue = remainder + 360;
        }

        return clampedValue;
    }

    public static double getDistance(Pose2d a, Pose2d b) {
        double difX = b.getX() - a.getX();
        double difY = b.getY() - a.getY();

        return Math.sqrt(difX * difX + difY * difY);
    }
}