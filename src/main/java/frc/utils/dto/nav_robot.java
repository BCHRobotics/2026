package frc.utils.dto;

import edu.wpi.first.wpilibj.util.Color;

public class nav_robot
{
    public int teamNumber;

    public double xPosition;
    public double yPosition;
    public double zPosition;

    public double zAngle;

    public double width; // (m)
    public double length; // (m)

    public Color color;

    public nav_robot() { }

    public nav_robot(int teamNumber, double xPosition, double yPosition, double zPosition, double zAngle, double width, double length)
    {
        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.zPosition = zPosition;

        this.teamNumber = teamNumber;

        this.zAngle = zAngle;
        this.width = width;
        this.length = length;

        this.color = Color.kWhite;
    }

    public static String EncodeToString(nav_robot input)
    {
        String result = "";

        result += "[";

        result += "{" + input.teamNumber + "}";

        result += "{[" + "{" + input.xPosition + "}" + "{" + input.yPosition + "}" + "{" + input.zPosition + "}" + "]}";
        result += "{" + input.zAngle + "}";

        result += "{" + input.width + "}";
        result += "{" + input.length + "}";

        result += "{" + "[" + "{" + input.color.red + "}" + "{" + input.color.green + "}" + "{" + input.color.blue + "}" + "]" + "}";

        result += "]";

        return "$" + result;
    }
}