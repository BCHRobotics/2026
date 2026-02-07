package frc.utils.dto;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.dto.m.Polygon;
import frc.utils.dto.m.Vector2;
import frc.utils.dto.m.Vector3;

public class nav_path
{
    public int drawMode; // this way paths can be used as polygons, trails, or just straight-up groups of points
    // USE THE ENUM TO REF

    public Vector3[] positions;

    // here we have colors for every individual point
    public Color[] pointColors;
    // and one color for all segments
    public Color segmentColor;



    public nav_path() {}

    public nav_path(Vector3[] positions)
    {
        this.positions = positions;
        SetColor(Color.kWhite);
    }
    public nav_path(Vector3[] positions, int drawMode)
    {
        this.positions = positions;
        this.drawMode = drawMode;

        SetColor(Color.kWhite);
    }
    public nav_path(Vector3[] positions, Color color, int drawMode)
    {
        this.positions = positions;
        this.drawMode = drawMode;

        pointColors = new Color[positions.length];
        for (int i = 0; i < positions.length; i++) { pointColors[i] = color; }
        segmentColor = color;
    }
    public nav_path(Vector3[] positions, Color color)
    {
        this.positions = positions;

        pointColors = new Color[positions.length];
        for (int i = 0; i < positions.length; i++) { pointColors[i] = color; }
        segmentColor = color;
    }
    // every color individual
    public nav_path(Vector3[] positions, Color segmentColor, Color[] pointColors, int drawMode)
    {
        this.positions = positions;
        this.drawMode = drawMode;

        this.pointColors = pointColors;
        this.segmentColor = segmentColor;
    }
    public nav_path(Vector3[] positions, Color segmentColor, Color[] pointColors)
    {
        this.positions = positions;

        this.pointColors = pointColors;
        this.segmentColor = segmentColor;
    }
    // this one is used when processing
    public nav_path(Pose2d[] poses) {
        this.positions = new Vector3[poses.length];
        for (int i = 0; i < poses.length; i++) {
            this.positions[i] = new Vector3(poses[i].getX(), poses[i].getY(), 0);
        }
        SetColor(Color.kBlue);
    }

    public void SetColor(Color color)
    {
        pointColors = new Color[positions.length];
        for (int i = 0; i < positions.length; i++) { pointColors[i] = color; }
        segmentColor = color;
    }

    // nav_path --> string (for uploading to NT)
    public static String EncodeToString(nav_path input)
    {
        String result = "";

        result += "[";

        result += "{" + input.drawMode + "}";

        result += "{" + "[" + "{" + input.segmentColor.red + "}" + "{" + input.segmentColor.green + "}" + "{" + input.segmentColor.blue + "}" + "]" + "}";

        result += "{";
        for (int i = 0; i < input.positions.length; i++)
        {
            result += "[" + "{" + input.positions[i].x + "}" + "{" + input.positions[i].y + "}" + "{" + input.positions[i].z + "}" + "]";
        }
        result += "}";

        result += "]";

        return "$" + result;
    }

    public static nav_path DecodeFromString(String input)
    {
        nav_path result = new nav_path();
        
        input = input.substring(1, input.length());

        result.drawMode = Integer.parseInt(input.substring(DTUtils.FindOccurance(input, "{", 0) + 1, DTUtils.FindClosingBracket(input, DTUtils.FindOccurance(input, "{", 0))));
        input = DTUtils.SubtractVariable(input);

        double r = Double.parseDouble(input.substring(DTUtils.FindOccurance(input, "{", 1) + 1, DTUtils.FindClosingBracket(input, DTUtils.FindOccurance(input, "{", 1))));
        double g = Double.parseDouble(input.substring(DTUtils.FindOccurance(input, "{", 2) + 1, DTUtils.FindClosingBracket(input, DTUtils.FindOccurance(input, "{", 2))));;
        double b = Double.parseDouble(input.substring(DTUtils.FindOccurance(input, "{", 3) + 1, DTUtils.FindClosingBracket(input, DTUtils.FindOccurance(input, "{", 3))));;

        // color
        input = DTUtils.SubtractVariable(input);

        List<Vector3> points = new LinkedList<Vector3>();

        int limitIndex = DTUtils.FindClosingBracket(input, 0);
        int currentIndex = DTUtils.FindOccurance(input, "[", 0);
        int numSkips = 0;
        while (currentIndex != -1 && currentIndex < limitIndex)
        {
            List<String> components = new LinkedList<String>();

            for (int i = 0; i < 3; i++)
            {
                int startIndex = DTUtils.FindOccurance(input, "{", i, currentIndex);
                // System.out.println("START" + startIndex);
                int endIndex = DTUtils.FindClosingBracket(input, startIndex);
                // System.out.println("END" + endIndex);

                components.add(input.substring(startIndex + 1, endIndex));
            }

            points.add(new Vector3(Double.parseDouble(components.get(0)), Double.parseDouble(components.get(1)), Double.parseDouble(components.get(2))));

            numSkips++;
            currentIndex = DTUtils.FindOccurance(input, "[", numSkips);
        }
        
        Vector3[] array = new Vector3[points.size()];
        result.positions = points.toArray(array);

        result.SetColor(new Color(r, g, b));

        return result;
    }

    // convert the positions of this path to poly format, for use in the pathfinding algorithm
    public Polygon ConvertToPolygon() {
        Polygon result = new Polygon();
        result.points = new Vector2[positions.length];

        for (int i = 0; i < positions.length; i++) {
            result.points[i] = new Vector2(positions[i].x, positions[i].y);
        }

        return result;
    }
}