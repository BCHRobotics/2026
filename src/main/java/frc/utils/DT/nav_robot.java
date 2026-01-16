/*
 * nav_robot.java
 * 
 * Robot Navigation State Representation for FRC Drivetools
 * 
 * This class encapsulates all navigation-relevant information about a robot on the field,
 * including its position, orientation, dimensions, and team identification.
 * 
 * Properties:
 * - teamNumber: FRC team number for robot identification
 * - xPosition, yPosition, zPosition: 3D position on the field (meters)
 * - zAngle: Robot heading/rotation around Z-axis (radians)
 * - width, length: Physical dimensions for collision detection (meters)
 * 
 * Key Use Cases:
 * - Real-time robot position tracking and visualization
 * - Multi-robot collision avoidance in practice/competition
 * - Path planning with robot footprint consideration
 * - Match replay and autonomous debugging
 * - Alliance coordination systems
 * 
 * Encoding Format:
 * Robots are encoded as: \"$[{teamNum}{[{x}{y}{z}]}{angle}{width}{length}]\"\n * This allows transmission via NetworkTables or logging for analysis tools.
 * 
 * @see nav_field For field-level robot aggregation
 * @see DTUtils For string encoding/decoding utilities
 */

package frc.utils.DT;

public class nav_robot
{
    /** FRC team number (e.g., 254, 1114, 1678) */
    public int teamNumber;

    /** X position on the field in meters (typically aligned with field length) */
    public double xPosition;
    
    /** Y position on the field in meters (typically aligned with field width) */
    public double yPosition;
    
    /** Z position on the field in meters (height above ground, usually 0) */
    public double zPosition;

    /** Robot heading - rotation around Z-axis in radians (0 = facing +X direction) */
    public double zAngle;

    /** Robot width in meters (for collision detection and visualization) */
    public double width;
    
    /** Robot length in meters (for collision detection and visualization) */
    public double length;

    /**
     * Default constructor - creates an uninitialized robot.
     * All properties should be set manually after construction.
     */
    public nav_robot() { }

    /**
     * Creates a robot with full navigation state.
     * 
     * @param teamNumber FRC team number (e.g., 254, 1678)
     * @param xPosition X coordinate on field in meters
     * @param yPosition Y coordinate on field in meters
     * @param zPosition Z coordinate (height) in meters, typically 0
     * @param zAngle Heading angle in radians (0 = facing +X)
     * @param width Robot width in meters (for collision bounds)
     * @param length Robot length in meters (for collision bounds)
     */
    public nav_robot(int teamNumber, double xPosition, double yPosition, double zPosition, double zAngle, double width, double length)
    {
        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.zPosition = zPosition;

        this.teamNumber = teamNumber;

        this.zAngle = zAngle;
        this.width = width;
        this.length = length;
    }

    /**
     * Encodes the robot's state into a string representation.
     * 
     * Format: "$[{teamNumber}{[{x}{y}{z}]}{zAngle}{width}{length}]"
     * 
     * This encoding allows the robot state to be:
     * - Transmitted via NetworkTables to driver station displays
     * - Logged for match replay and analysis
     * - Sent to visualization tools (Drivetools Dashboard, etc.)
     * - Shared between robots for multi-robot coordination
     * 
     * The nested bracket structure ensures proper parsing even with floating-point values.
     * 
     * @param input The robot to encode
     * @return String encoding with '$' prefix indicating navigation data
     * 
     * Example output: "$[{1678}{[{2.5}{3.0}{0.0}]}{1.57}{0.8}{0.9}]"
     */
    public static String EncodeToString(nav_robot input)
    {
        String result = "";

        result += "[";

        // Team identification
        result += "{" + input.teamNumber + "}";

        // 3D position as nested structure
        result += "{[" + "{" + input.xPosition + "}" + "{" + input.yPosition + "}" + "{" + input.zPosition + "}" + "]}";
        
        // Heading angle
        result += "{" + input.zAngle + "}";

        // Physical dimensions for collision detection
        result += "{" + input.width + "}";
        result += "{" + input.length + "}";

        result += "]";

        // Add '$' prefix to indicate encoded navigation data
        return "$" + result;
    }
}