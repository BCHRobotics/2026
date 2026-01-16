/*
 * Polygon.java
 * 
 * 2D Polygonal Shape Representation for FRC Pathfinding
 * 
 * This class represents a closed 2D polygon defined by a series of vertices.
 * It's specifically designed for use in pathfinding algorithms that navigate
 * around obstacles and restricted zones on the FRC field.
 * 
 * Key Features:
 * - Vertex-based polygon definition using Vector2 points
 * - Centroid (midpoint) calculation for polygon center
 * - Integration with pathfinding algorithms
 * 
 * Use Cases:
 * - Obstacle representation for A* pathfinding
 * - Field boundary definitions
 * - Restricted zone marking (opponent's loading zones, etc.)
 * - Safe navigation area definitions
 * - Collision detection boundaries
 * 
 * Relationship to nav_path:
 * While nav_path.java handles visualization and trajectory representation,
 * Polygon.java focuses on geometric shape definitions for pathfinding logic.
 * The separation keeps visualization concerns separate from pathfinding mathematics.
 * 
 * @see Vector2 For 2D point representation
 * @see nav_path For path visualization (can convert to Polygon)
 */

package frc.utils.M;

public class Polygon {
    /** Array of vertices defining the polygon boundary (ordered sequence) */
    public Vector2[] points;

    /**
     * Default constructor - creates an empty polygon.
     * Points array should be initialized separately.
     */
    public Polygon() {}

    /**
     * Creates a polygon from an array of vertices.
     * 
     * @param points Array of Vector2 points defining the polygon vertices
     *               (should be ordered clockwise or counter-clockwise)
     */
    public Polygon(Vector2[] points) {
        this.points = points;
    }
    
    /**
     * Calculates the centroid (geometric center) of the polygon.
     * 
     * The centroid is computed as the average of all vertex positions.
     * This works well for convex polygons and provides a reasonable
     * approximation for concave polygons.
     * 
     * Algorithm:
     * 1. Sum all vertex positions
     * 2. Divide by number of vertices
     * 
     * @return Vector2 representing the polygon's center point
     * 
     * Use cases:
     * - Finding center of obstacles for distance calculations
     * - Determining polygon reference point for pathfinding
     * - Calculating waypoints that avoid polygon centers
     * - Visualizing polygon centers in debugging displays
     * 
     * Note: For complex concave shapes, this may not be the true
     *       geometric centroid, but it's sufficient for most pathfinding needs.
     */
    public Vector2 GetMidpoint() {
        Vector2 midpoint = new Vector2(0, 0);
        
        // Sum all vertex positions
        for (int i = 0; i < points.length; i++) {
            midpoint = Vector2.add(midpoint, points[i]);
        }
        
        // Divide by count to get average position
        midpoint = new Vector2(midpoint.x / points.length, midpoint.y / points.length);
        return midpoint;
    }
}