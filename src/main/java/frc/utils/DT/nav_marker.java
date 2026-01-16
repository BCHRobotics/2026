/*
 * nav_marker.java
 * 
 * Navigation Marker Class for FRC Field Localization
 * 
 * Represents a fixed point of interest on the FRC field, such as:
 * - Game piece locations (notes, coral, etc.)
 * - Scoring targets (speaker, amp, reef)
 * - Reference points for localization
 * - Autonomous waypoints
 * 
 * Structure:
 * - position: 3D position on the field (Vector3)
 * - zAngle: Rotation around the Z-axis (heading) in radians
 * 
 * This class serves as a fundamental building block for:
 * - Field mapping and visualization
 * - Autonomous navigation planning
 * - Vision target tracking
 * - Path planning obstacle avoidance
 * 
 * Note: Designed to avoid class duplication with other navigation systems.
 * 
 * @see Vector3 For 3D position representation
 * @see nav_path For navigation trajectories
 * @see nav_field For field-wide navigation structure
 */

package frc.utils.DT;

import frc.utils.M.Vector3;

public class nav_marker {
    /** 3D position of the marker on the field (x, y, z in meters) */
    public Vector3 position;
    
    /** Rotation angle around the Z-axis (heading) in radians */
    public double zAngle;

    /**
     * Default constructor - creates an uninitialized marker.
     * Position and angle should be set manually after construction.
     */
    public nav_marker() {}

    /**
     * Creates a marker at a specific position and orientation.
     * 
     * @param position 3D position on the field (typically z=0 for ground level)
     * @param zAngle Rotation around the Z-axis in radians (0 = facing +X direction)
     */
    public nav_marker(Vector3 position, double zAngle) {
        this.position = position;
        this.zAngle = zAngle;
    }

    /**
     * Gets the X coordinate of the marker's position.
     * @return X position in meters
     */
    public double getX() {
        return position.x;
    }
    
    /**
     * Gets the Y coordinate of the marker's position.
     * @return Y position in meters
     */
    public double getY() {
        return position.y;
    }
    
    /**
     * Gets the Z coordinate of the marker's position.
     * @return Z position in meters (typically 0 for ground-level markers)
     */
    public double getZ() {
        return position.z;
    }

    /**
     * Calculates the Euclidean distance between this marker and another marker.
     * 
     * Uses the 3D distance formula: sqrt((x2-x1)² + (y2-y1)² + (z2-z1)²)
     * 
     * This is useful for:
     * - Finding the nearest marker to a robot's current position
     * - Calculating travel distances for path planning
     * - Determining if a marker is within operational range
     * 
     * @param other The marker to calculate distance to
     * @return Distance in meters between the two markers
     */
    public double getDistance(nav_marker other) {
        // Calculate component differences
        double xDiff = other.position.x - position.x;
        double yDiff = other.position.y - position.y;
        double zDiff = other.position.z - position.z;

        // Apply Pythagorean theorem in 3D
        return Math.sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);
    }
}