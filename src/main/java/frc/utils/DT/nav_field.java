/*
 * nav_field.java
 * 
 * Field Navigation Structure for FRC Robot Drivetools
 * 
 * This class represents the top-level navigation data structure for the entire FRC field.
 * It aggregates all navigation entities (robots, paths, markers) into a single coherent
 * field representation that can be serialized and transmitted to visualization tools.
 * 
 * Structure:
 * - robots: Array of nav_robot objects representing all robots on the field
 * - paths: Array of nav_path objects for navigation trajectories (currently commented out)
 * - markers: Array of nav_marker objects for field landmarks (currently commented out)
 * 
 * Encoding Format:
 * The field is encoded as: "field(robots{[robot1][robot2]...}paths{[path1]...}markers{[marker1]...})"
 * This string format allows the field state to be transmitted via NetworkTables or logged to files.
 * 
 * Use Cases:
 * - Real-time field visualization in driver stations
 * - Autonomous path planning and debugging
 * - Multi-robot coordination systems
 * - Match replay and analysis tools
 * 
 * @see nav_robot For individual robot position and dimension data
 * @see nav_path For trajectory and waypoint data (future implementation)
 * @see nav_marker For field landmark data (future implementation)
 * @see DTUtils For string encoding/decoding utilities
 */

package frc.utils.DT;

public class nav_field
{
    /** Array of all robots currently tracked on the field */
    public nav_robot[] robots;
    
    // Future implementation - path tracking for autonomous navigation
    // public nav_path[] paths;
    
    // Future implementation - field markers for localization and navigation
    // public nav_marker[] markers;

    /**
     * Encodes this field instance to a string representation.
     * 
     * Convenience wrapper that calls the static encoding method on this object.
     * Useful for method chaining and object-oriented code patterns.
     * 
     * @return String encoding of this field in format: "$field(robots{...}paths{...}markers{...})"
     */
    public String EncodeToString()
    {
        return nav_field.EncodeToString(this);
    }

    /**
     * Encodes a nav_field object into a string representation for transmission/storage.
     * 
     * This static method performs the actual encoding work. The resulting string can be:
     * - Sent over NetworkTables to visualization tools
     * - Logged to files for match analysis
     * - Transmitted to driver stations for real-time field display
     * 
     * Encoding Format:
     * "$field(robots{[robot1][robot2]...}paths{...}markers{...})"
     * 
     * The '$' prefix indicates this is an encoded navigation structure.
     * Each section (robots, paths, markers) contains bracketed arrays of encoded objects.
     * 
     * @param input The nav_field object to encode
     * @return Complete string representation with '$' prefix
     * 
     * Note: paths and markers sections are currently empty (commented out in implementation)
     *       but the structure is maintained for future expansion.
     */
    public static String EncodeToString(nav_field input)
    {
        // Encoding structure: "field(robots{[]}paths{[]}markers{[]})"

        String result = "";

        result += "field(";

        // *** Encode all robots on the field ***
        result += "robots{";
        for (int i = 0; i < input.robots.length; i++)
        {
            // Each robot encodes itself to a bracketed string format
            result += nav_robot.EncodeToString(input.robots[i]);
        }
        result += "}";

        // *** Encode all paths (future implementation) ***
        result += "paths{";
        // for (int i = 0; i < input.paths.Length; i++)
        // {
        //     result += nav_path.EncodeToString(input.paths[i]);
        // }
        result += "}";

        // *** Encode all markers (future implementation) ***
        result += "markers{";
        // for (int i = 0; i < input.markers.Length; i++)
        // {
        //     result += nav_marker.EncodeToString(input.markers[i]);
        // }
        result += "}";

        result += ")";

        // Add '$' prefix to indicate encoded navigation data
        return "$" + result;
    }
}
