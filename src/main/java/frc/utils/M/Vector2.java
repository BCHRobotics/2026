/*
 * Vector2.java
 * 
 * 2D Vector Mathematics Class for FRC Robot Navigation
 * 
 * This class provides a complete 2D vector implementation for mathematical operations
 * commonly needed in robotics applications, particularly for field navigation and pathfinding.
 * 
 * Core Operations:
 * - Vector addition and normalization
 * - Distance and magnitude calculations
 * - Dot product and angle computation
 * - Linear interpolation (lerp) for smooth path generation
 * 
 * Use Cases:
 * - 2D position representation on the FRC field
 * - Velocity and acceleration vectors for motion profiling
 * - Direction vectors for pathfinding algorithms
 * - Polygon vertices for obstacle representation
 * 
 * Design Note:
 * Currently uses static methods for most operations. There's a TODO to convert
 * normalize() and magnitude() to instance methods for better OOP design.
 * 
 * @see Vector3 For 3D vector operations
 * @see Polygon For 2D shape representation using Vector2 points
 */

package frc.utils.M;

public class Vector2 {
    /** X coordinate (horizontal position on field) */
    public double x;
    
    /** Y coordinate (vertical position on field) */
    public double y;

    /**
     * Default constructor - creates a zero vector (0, 0).
     */
    public Vector2() {}

    /**
     * Creates a vector with specified coordinates.
     * 
     * @param x X coordinate (horizontal)
     * @param y Y coordinate (vertical)
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // TODO: make the following functions not static
    /*
     * normalize()
     * magnitude()
     */

    /**
     * Adds two vectors component-wise.
     * 
     * Result = (a.x + b.x, a.y + b.y)
     * 
     * Used for:
     * - Combining position and displacement vectors
     * - Accumulating forces or velocities
     * - Translating points in 2D space
     * 
     * @param a First vector
     * @param b Second vector
     * @return New vector representing the sum
     */
    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    /**
     * Normalizes a vector to unit length (magnitude = 1).
     * 
     * The resulting vector points in the same direction but has length 1.
     * This is useful for direction vectors where only the direction matters.
     * 
     * Formula: normalized = vector / magnitude(vector)
     * 
     * @param a Vector to normalize
     * @return Unit vector in the same direction
     * 
     * Warning: Will produce NaN values if the input vector is (0, 0)
     */
    public static Vector2 normalize(Vector2 a) {
        double length = Math.sqrt(a.x * a.x + a.y * a.y);
        return new Vector2(a.x / length, a.y / length);
    }

    /**
     * Calculates the magnitude (length) of a vector.
     * 
     * Uses the Pythagorean theorem: magnitude = √(x² + y²)
     * 
     * @param a Vector to measure
     * @return Length of the vector in the same units as its components
     * 
     * Use cases:
     * - Finding distance from origin
     * - Calculating speed from velocity vector
     * - Normalizing vectors (dividing by magnitude)
     */
    public static double magnitude(Vector2 a) {
        return Math.sqrt(a.x * a.x + a.y * a.y);
    }

    /**
     * Calculates the Euclidean distance between two points.
     * 
     * Formula: distance = sqrt((x2-x1)^2 + (y2-y1)^2)
     * 
     * @param a First point
     * @param b Second point
     * @return Distance between the points in the same units as coordinates
     * 
     * Common uses:
     * - Pathfinding distance calculations
     * - Proximity detection for game pieces
     * - Range checking for robot mechanisms
     */
    public static double distance(Vector2 a, Vector2 b) {
        return Math.sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
    }

    /**
     * Calculates the dot product of two vectors.
     * 
     * Formula: dot(a, b) = a.x * b.x + a.y * b.y
     * 
     * The dot product has important geometric meanings:
     * - If result > 0: vectors point in similar directions
     * - If result = 0: vectors are perpendicular
     * - If result < 0: vectors point in opposite directions
     * 
     * @param a First vector
     * @param b Second vector
     * @return Scalar dot product value
     * 
     * Used for:
     * - Calculating angle between vectors (with angleBetween)
     * - Projecting one vector onto another
     * - Determining if robot is facing a target
     */
    public static double dot(Vector2 a, Vector2 b) {
        return a.x * b.x + a.y * b.y;
    }
    /**
     * Calculates the angle between two vectors in radians.
     * 
     * Formula: θ = arccos(dot(a,b) / (|a| * |b|))
     * 
     * @param a First vector
     * @param b Second vector
     * @return Angle between vectors in radians [0, π]
     * 
     * Applications:
     * - Determining robot heading error to target
     * - Calculating turn angles for pathfinding
     * - Vision alignment angle calculations
     * 
     * Note: Result is always positive (0 to π radians)
     */
    public static double angleBetween(Vector2 a, Vector2 b) {
        return Math.acos(dot(a, b) / Vector2.magnitude(a) / Vector2.magnitude(b));
    }

    /**
     * Performs linear interpolation between two vectors.
     * 
     * Formula: lerp = a + (b - a) * c
     * 
     * When c = 0: returns a
     * When c = 1: returns b
     * When c = 0.5: returns midpoint
     * 
     * @param a Start vector
     * @param b End vector
     * @param c Interpolation factor [0..1]
     * @return Interpolated vector between a and b
     * 
     * Use cases:
     * - Smooth path generation between waypoints
     * - Animation and motion profiling
     * - Gradual position transitions
     * 
     * Note: c values outside [0,1] will extrapolate beyond the range
     */
    public static Vector2 lerp(Vector2 a, Vector2 b, double c) {
        return new Vector2(a.x + (b.x-a.x) * c, a.y + (b.y-a.y)*c);
    }

    /**
     * Checks if two vectors are exactly equal.
     * 
     * Compares both x and y components using exact equality (==).
     * 
     * @param a First vector
     * @param b Second vector
     * @return true if both components are exactly equal, false otherwise
     * 
     * Warning: Due to floating-point precision issues, exact equality checks
     *          may fail for computed values. Consider using an epsilon-based
     *          comparison for fuzzy equality in most practical applications.
     */
    public static boolean equals(Vector2 a, Vector2 b) {
        if (a.x == b.x && a.y == b.y) {
            return true;
        } else {return false;}
    }
}