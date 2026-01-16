/*
 * nav_path.java
 * 
 * Navigation Path Representation for FRC Robot Drivetools
 * 
 * This class represents a path as a collection of waypoints with visualization properties.
 * It serves multiple purposes in the FRC robot navigation system:
 * 
 * Primary Functions:
 * - Path visualization in driver stations and debugging tools
 * - Autonomous trajectory representation
 * - Field boundary and obstacle definitions
 * - Real-time robot trajectory tracking
 * 
 * Drawing Modes:
 * Paths can be visualized in different ways based on drawMode:
 * - Points only: Display individual waypoints
 * - Lines/Trail: Connect waypoints with line segments
 * - Polygon: Closed shape for zones or obstacles
 * 
 * Color System:
 * - pointColors: Individual color for each waypoint
 * - segmentColor: Color for lines connecting waypoints
 * This allows rich visualization (e.g., gradient paths, highlighted waypoints)
 * 
 * Encoding/Decoding:
 * Paths can be serialized to strings for NetworkTables transmission and
 * deserialized for visualization in external tools.
 * 
 * Integration:
 * - PathPlanner: Converts PathPlanner Pose2d arrays to nav_path
 * - Polygon: Can export to Polygon format for pathfinding algorithms
 * - WPILib: Compatible with standard FRC geometry classes
 * 
 * @see nav_field For field-level path aggregation
 * @see Vector3 For 3D waypoint positions
 * @see Polygon For pathfinding polygon conversion
 * @see DTUtils For string encoding/decoding
 */

package frc.utils.DT;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.M.Polygon;
import frc.utils.M.Vector2;
import frc.utils.M.Vector3;

public class nav_path
{
    /**
     *      * Drawing mode for path visualization.
     *      * Use enum constants to reference (e.g., POINTS, TRAIL, POLYGON)
     *      */
    public int drawMode;

    /** Array of 3D positions defining the path waypoints */
    public Vector3[] positions;

    /** Color for each individual waypoint (same length as positions array) */
    public Color[] pointColors;   /** Color for line segments connecting waypoints */
    public Color segmentColor;



    /**
     * Default constructor - creates an uninitialized path.
     * Properties should be set manually after construction.
     */
    public nav_path() {}

    /**
     * Creates a path from waypoint positions with default white color.
     * 
     * @param positions Array of 3D waypoint positions
     */
    public nav_path(Vector3[] positions)
    {
        this.positions = positions;
        SetColor(Color.kWhite);
    }
    /**
     * Creates a path with specified visualization mode and default white color.
     * 
     * @param positions Array of 3D waypoint positions
     * @param drawMode Visualization mode (POINTS, TRAIL, POLYGON, etc.)
     */
    public nav_path(Vector3[] positions, int drawMode)
    {
        this.positions = positions;
        this.drawMode = drawMode;

        SetColor(Color.kWhite);
    }
    /**
     * Creates a path with uniform color for all points and segments.
     * 
     * @param positions Array of 3D waypoint positions
     * @param color Single color to apply to all points and segments
     * @param drawMode Visualization mode
     */
    public nav_path(Vector3[] positions, Color color, int drawMode)
    {
        this.positions = positions;
        this.drawMode = drawMode;

        // Apply same color to all points
        pointColors = new Color[positions.length];
        for (int i = 0; i < positions.length; i++) { pointColors[i] = color; }
        segmentColor = color;
    }
    /**
     * Creates a path with uniform color (no drawMode specified).
     * 
     * @param positions Array of 3D waypoint positions
     * @param color Single color to apply to all points and segments
     */
    public nav_path(Vector3[] positions, Color color)
    {
        this.positions = positions;

        // Apply same color to all points
        pointColors = new Color[positions.length];
        for (int i = 0; i < positions.length; i++) { pointColors[i] = color; }
        segmentColor = color;
    }
    /**
     * Creates a path with individual colors for each point (maximum flexibility).
     * 
     * This constructor allows for gradient effects, color-coded waypoints by priority,
     * or any other custom visualization scheme.
     * 
     * @param positions Array of 3D waypoint positions
     * @param segmentColor Color for line segments connecting points
     * @param pointColors Array of colors (one per waypoint, must match positions length)
     * @param drawMode Visualization mode
     */
    public nav_path(Vector3[] positions, Color segmentColor, Color[] pointColors, int drawMode)
    {
        this.positions = positions;
        this.drawMode = drawMode;

        this.pointColors = pointColors;
        this.segmentColor = segmentColor;
    }
    /**
     * Creates a path with individual colors for each point (no drawMode specified).
     * 
     * @param positions Array of 3D waypoint positions
     * @param segmentColor Color for line segments connecting points
     * @param pointColors Array of colors (one per waypoint)
     */
    public nav_path(Vector3[] positions, Color segmentColor, Color[] pointColors)
    {
        this.positions = positions;

        this.pointColors = pointColors;
        this.segmentColor = segmentColor;
    }
    /**
     * Creates a path from WPILib Pose2d array (PathPlanner integration).
     * 
     * This constructor is specifically designed for processing PathPlanner trajectories.
     * It extracts X and Y positions from each pose (Z is set to 0) and creates
     * a visualization-ready path with blue coloring.
     * 
     * @param poses Array of Pose2d from PathPlanner trajectory or other WPILib source
     */
    public nav_path(Pose2d[] poses) {
        // Convert Pose2d to Vector3 (extracting X, Y, setting Z=0)
        this.positions = new Vector3[poses.length];
        for (int i = 0; i < poses.length; i++) {
            this.positions[i] = new Vector3(poses[i].getX(), poses[i].getY(), 0);
        }
        // Default blue color for autonomous paths
        SetColor(Color.kBlue);
    }

    /**
     * Sets a uniform color for all points and segments in the path.
     * 
     * Useful for quickly changing path visualization without reconstructing the object.
     * 
     * @param color Color to apply to all waypoints and connecting segments
     */
    public void SetColor(Color color)
    {
        pointColors = new Color[positions.length];
        for (int i = 0; i < positions.length; i++) { pointColors[i] = color; }
        segmentColor = color;
    }

    /**
     * Encodes a path to string format for NetworkTables or logging.
     * 
     * Format: "$[{drawMode}{[{r}{g}{b}]}{[{x}{y}{z}][{x}{y}{z}]...}]"
     * 
     * The encoded string contains:
     * - drawMode: Visualization type
     * - Segment color: RGB values (0.0-1.0)
     * - All waypoint positions: Each as {x}{y}{z}
     * 
     * Note: Individual point colors are not currently encoded (uses segmentColor for all)
     * 
     * @param input The path to encode
     * @return String encoding with '$' prefix
     */
    public static String EncodeToString(nav_path input)
    {
        String result = "";

        result += "[";

        // Encode draw mode
        result += "{" + input.drawMode + "}";

        // Encode segment color as RGB
        result += "{" + "[" + "{" + input.segmentColor.red + "}" + "{" + input.segmentColor.green + "}" + "{" + input.segmentColor.blue + "}" + "]" + "}";

        // Encode all waypoint positions
        result += "{";
        for (int i = 0; i < input.positions.length; i++)
        {
            result += "[" + "{" + input.positions[i].x + "}" + "{" + input.positions[i].y + "}" + "{" + input.positions[i].z + "}" + "]";
        }
        result += "}";

        result += "]";

        return "$" + result;
    }

    /**
     * Decodes a string-encoded path back into a nav_path object.
     * 
     * This method reverses the encoding process, parsing the bracketed string format
     * to reconstruct the path with all its properties.
     * 
     * Parsing Process:
     * 1. Removes leading '$' indicator
     * 2. Extracts drawMode from first bracketed value
     * 3. Parses RGB color components for segment color
     * 4. Iterates through position brackets, extracting X, Y, Z for each waypoint
     * 5. Applies parsed color to all points via SetColor()
     * 
     * @param input Encoded string (format: \"$[{drawMode}{color}{positions...}]\")
     * @return Reconstructed nav_path object
     * 
     * Used for:
     * - Receiving paths from NetworkTables
     * - Loading paths from log files
     * - Synchronizing paths between robots or tools
     */
    public static nav_path DecodeFromString(String input)
    {
        nav_path result = new nav_path();

        // Remove leading '$' prefix
        input = input.substring(1, input.length());

        // Parse drawMode (first bracketed value)
        result.drawMode = Integer.parseInt(input.substring(DTUtils.FindOccurance(input, "{", 0) + 1, DTUtils.FindClosingBracket(input, DTUtils.FindOccurance(input, "{", 0))));
        input = DTUtils.SubtractVariable(input);

        // Parse RGB color components
        double r = Double.parseDouble(input.substring(DTUtils.FindOccurance(input, "{", 1) + 1, DTUtils.FindClosingBracket(input, DTUtils.FindOccurance(input, "{", 1))));
        double g = Double.parseDouble(input.substring(DTUtils.FindOccurance(input, "{", 2) + 1, DTUtils.FindClosingBracket(input, DTUtils.FindOccurance(input, "{", 2))));;
        double b = Double.parseDouble(input.substring(DTUtils.FindOccurance(input, "{", 3) + 1, DTUtils.FindClosingBracket(input, DTUtils.FindOccurance(input, "{", 3))));;

        // Remove color section from input
        input = DTUtils.SubtractVariable(input);

        // Parse all position waypoints
        List<Vector3> points = new LinkedList<Vector3>();

        int limitIndex = DTUtils.FindClosingBracket(input, 0);
        int currentIndex = DTUtils.FindOccurance(input, "[", 0);
        int numSkips = 0;
        
        // Loop through all position brackets until we exceed the limit
        while (currentIndex != -1 && currentIndex < limitIndex)
        {
            List<String> components = new LinkedList<String>();

            // Extract X, Y, Z components for this waypoint
            for (int i = 0; i < 3; i++)
            {
                int startIndex = DTUtils.FindOccurance(input, "{", i, currentIndex);
                int endIndex = DTUtils.FindClosingBracket(input, startIndex);

                components.add(input.substring(startIndex + 1, endIndex));
            }

            // Create Vector3 from parsed components
            points.add(new Vector3(Double.parseDouble(components.get(0)), Double.parseDouble(components.get(1)), Double.parseDouble(components.get(2))));

            // Move to next position bracket
            numSkips++;
            currentIndex = DTUtils.FindOccurance(input, "[", numSkips);
        }
        
        // Convert list to array
        Vector3[] array = new Vector3[points.size()];
        result.positions = points.toArray(array);

        // Apply parsed color to all points
        result.SetColor(new Color(r, g, b));

        return result;
    }

    /**
     * Converts this path to a Polygon for pathfinding algorithms.
     * 
     * Extracts the 2D positions (X, Y) from the 3D waypoints, discarding Z values.
     * This is useful for:
     * - A* pathfinding obstacle representation
     * - Collision detection with field boundaries
     * - Zone definitions (scoring areas, restricted zones, etc.)
     * 
     * The resulting polygon can be used with pathfinding algorithms that operate
     * on 2D field representations.
     * 
     * @return Polygon object with 2D points matching this path's waypoints
     */
    public Polygon ConvertToPolygon() {
        Polygon result = new Polygon();
        result.points = new Vector2[positions.length];

        // Extract X and Y coordinates, discarding Z
        for (int i = 0; i < positions.length; i++) {
            result.points[i] = new Vector2(positions[i].x, positions[i].y);
        }

        return result;
    }
}