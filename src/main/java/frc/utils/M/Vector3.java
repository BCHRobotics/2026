/*
 * Vector3.java
 * 
 * 3D Vector Mathematics Class for FRC Robot Navigation
 * 
 * This class provides 3D vector operations for representing positions, directions,
 * and transformations in 3D space. It's the Java equivalent of "DoubleVector3" from
 * the C# Drivetools implementation.
 * 
 * Core Capabilities:
 * - 3D position and direction representation
 * - Vector addition (static and instance methods)
 * - 2D rotation operations (X-Y plane)
 * - String encoding/decoding for NetworkTables transmission
 * - WPILib Pose2d/Pose3d integration
 * 
 * Use Cases:
 * - Robot position on field with elevation (though z is often 0)
 * - 3D game piece tracking
 * - Camera/vision target positions
 * - Path waypoints with height information
 * - AprilTag 3D position representation
 * 
 * Encoding Format:
 * Vectors encode as: "$vector3({x}{y}{z})" for NetworkTables transmission
 * 
 * @see Vector2 For 2D vector operations
 * @see nav_path For using Vector3 in navigation paths
 * @see DTUtils For string encoding/decoding utilities
 */

package frc.utils.M;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.utils.DT.DTUtils;

public class Vector3 {
    /** X coordinate (typically field length direction) */
    public double x;
    
    /** Y coordinate (typically field width direction) */
    public double y;
    
    /** Z coordinate (height above ground, often 0 for ground-level robots) */
    public double z;

    /**
     * Default constructor - creates a zero vector (0, 0, 0).
     */
    public Vector3() {}

    /**
     * Creates a vector with specified 3D coordinates.
     * 
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate (height)
     */
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    /**
     * Creates a Vector3 from a WPILib Pose2d (2D position).
     * 
     * Extracts X and Y from the pose, sets Z to 0 (ground level).
     * Useful for converting odometry positions to Vector3 format.
     * 
     * @param p Pose2d from odometry or pathfinding
     */
    public Vector3(Pose2d p) {
        this.x = p.getX();
        this.y = p.getY();
        this.z = 0;
    }
    /**
     * Creates a Vector3 from a WPILib Pose3d (3D position).
     * 
     * Extracts X, Y, and Z from the 3D pose.
     * Useful for vision targets or 3D AprilTag positions.
     * 
     * @param p Pose3d from vision system or 3D tracking
     */
    public Vector3(Pose3d p) {
        this.x = p.getX();
        this.y = p.getY();
        this.z = p.getZ();
    }

    /**
     * Adds two vectors component-wise (static method).
     * 
     * Result = (a.x + b.x, a.y + b.y, a.z + b.z)
     * 
     * @param a First vector
     * @param b Second vector
     * @return New vector representing the sum
     */
    public static Vector3 Add(Vector3 a, Vector3 b) {
        return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    /**
     * Adds another vector to this vector (instance method).
     * 
     * Returns a new vector; does not modify this instance.
     * 
     * @param b Vector to add
     * @return New vector representing this + b
     */
    public Vector3 Add(Vector3 b) {
        return new Vector3(x + b.x, y + b.y, z + b.z);
    }

    /**
     * Encodes a Vector3 to string format for NetworkTables transmission.
     * 
     * Format: "$vector3({x}{y}{z})"
     * 
     * The '$' prefix indicates this is an encoded navigation data structure.
     * 
     * @param input Vector to encode
     * @return String encoding suitable for NetworkTables or logging
     */
    public static String EncodeToString(Vector3 input)
    {
        String result = "";

        result += "vector3(";

        result += "{" + input.x + "}" + "{" + input.y + "}" + "{" + input.z + "}";

        result += ")";

        return "$" + result;
    }
    /**
     * Decodes a string-encoded Vector3 back to a Vector3 object.
     * 
     * Parses the bracketed format to extract X, Y, Z components.
     * 
     * Process:
     * 1. Find each component's bracketed section
     * 2. Extract the numeric value from within brackets
     * 3. Parse as double and construct Vector3
     * 
     * @param input Encoded string (format: "$vector3({x}{y}{z})")
     * @return Decoded Vector3 object
     * 
     * Used for receiving vectors from NetworkTables or log files.
     */
    public static Vector3 DecodeFromString(String input)
    {
        List<String> components = new LinkedList<String>();

        // Extract the three bracketed values
        for (int i = 1; i < 4; i++)
        {
            int startIndex = DTUtils.FindOccurance(input, "{", i);
            int endIndex = DTUtils.FindClosingBracket(input, startIndex);

            components.add(input.substring(startIndex + 1, endIndex));
        }

        return new Vector3(Double.parseDouble(components.get(0)), Double.parseDouble(components.get(1)), Double.parseDouble(components.get(2)));
    }

    /**
     * Rotates the vector in the X-Y plane by an angle in degrees.
     * 
     * Only rotates X and Y components; Z remains unchanged.
     * Uses standard 2D rotation matrix:
     *   x' = x*cos(θ) - y*sin(θ)
     *   y' = y*cos(θ) + x*sin(θ)
     * 
     * @param theta Rotation angle in degrees (positive = counter-clockwise)
     * @return New rotated vector
     * 
     * Note: Z is set to 0 in the result (verify if this is intentional)
     */
    public Vector3 RotateDeg(double theta) {
        double a = theta * Math.PI / 180;
        return new Vector3(x * Math.cos(a) - y * Math.sin(a), y * Math.cos(a) + x * Math.sin(a), 0);
    }
    
    /**
     * Rotates the vector in the X-Y plane by an angle in radians.
     * 
     * Only rotates X and Y components; Z remains unchanged.
     * 
     * @param theta Rotation angle in radians (positive = counter-clockwise)
     * @return New rotated vector
     * 
     * Use cases:
     * - Transforming vectors between robot-relative and field-relative coordinates
     * - Rotating waypoints around a pivot point
     * - Vision target angle corrections
     */
    public Vector3 RotateRad(double theta) {
        double a = theta;
        return new Vector3(x * Math.cos(a) - y * Math.sin(a), y * Math.cos(a) + x * Math.sin(a), 0);
    }
}