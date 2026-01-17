/*
 * Vector4.java
 * 
 * 4D Vector Mathematics Class for FRC Applications
 * 
 * This class provides a 4D vector implementation, primarily used for:
 * - Quaternion representation (x, y, z, w) for 3D rotations
 * - Homogeneous coordinates in computer graphics
 * - RGBA color representation with alpha channel
 * - Extended state vectors in control systems
 * 
 * Current Implementation:
 * Currently provides basic 4D vector storage and addition.
 * Additional operations can be added as needed for specific use cases.
 * 
 * Typical Use Cases in FRC:
 * - Quaternion-based robot orientation (avoiding gimbal lock)
 * - Color values with transparency for visualization
 * - 4D state space representations in advanced control algorithms
 * 
 * @see Vector3 For 3D vector operations (most common for FRC)
 * @see Vector2 For 2D vector operations
 */

package frc.utils.M;

public class Vector4 {
    /** X component (or i component for quaternions, or Red for RGBA) */
    public double x;
    
    /** Y component (or j component for quaternions, or Green for RGBA) */
    public double y;
    
    /** Z component (or k component for quaternions, or Blue for RGBA) */
    public double z;
    
    /** W component (scalar part for quaternions, or Alpha for RGBA) */
    public double w;

    /**
     * Default constructor - creates a zero vector (0, 0, 0, 0).
     */
    public Vector4() {}

    /**
     * Creates a vector with specified 4D coordinates.
     * 
     * @param x X component
     * @param y Y component
     * @param z Z component
     * @param w W component
     */
    public Vector4(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Adds two 4D vectors component-wise.
     * 
     * Result = (a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w)
     * 
     * Note: This is standard vector addition. For quaternion multiplication
     *       (which is different), a separate method would be needed.
     * 
     * @param a First vector
     * @param b Second vector
     * @return New vector representing the component-wise sum
     */
    public static Vector4 Add(Vector4 a, Vector4 b) {
        return new Vector4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
    }
}