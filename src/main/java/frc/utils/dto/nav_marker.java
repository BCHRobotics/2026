package frc.utils.dto;

import edu.wpi.first.wpilibj.util.Color;
import frc.utils.dto.m.Vector3;

/*
 * trying to make sure I don't have any duplicate classes here
 */

public class nav_marker {
    public int drawIndex;

    public Vector3 position;
    public double zAngle;

    public double size;

    public Color color;

    // there IS a color variable on the c# side, but I don't rlly care abt that here
    

    public nav_marker() {}

    public nav_marker(Vector3 position, double zAngle) {
        this.position = position;
        this.zAngle = zAngle;
        this.drawIndex = 0;
        this.size = 1;
        this.color = new Color(1,1,1);
    }
    public nav_marker(Vector3 position, double zAngle, int drawIndex) {
        this.position = position;
        this.zAngle = zAngle;
        this.drawIndex = drawIndex;
        this.size = 1;
        this.color = new Color(1,1,1);
    }
    public nav_marker(Vector3 position, double zAngle, int drawIndex, double size) {
        this.position = position;
        this.zAngle = zAngle;
        this.drawIndex = drawIndex;
        this.size = size;
        this.color = new Color(1,1,1);
    }
    public nav_marker(Vector3 position, double zAngle, int drawIndex, double size, Color color) {
        this.position = position;
        this.zAngle = zAngle;
        this.drawIndex = drawIndex;
        this.size = size;
        this.color = color;
    }

    public static String EncodeToString(nav_marker input)
    {
        String result = "";

        result += "[";

        result += "{" + input.drawIndex + "}";

        result += "{[" + "{" + input.position.x + "}" + "{" + input.position.y + "}" + "{" + input.position.z + "}" + "]}";
        result += "{" + input.zAngle + "}";
        result += "{" + input.size + "}";

        // no color, just putting (1,1,1) bc I have to put SOMETHING
        result += "{" + "[" + "{" + input.color.red + "}" + "{" + input.color.green + "}" + "{" + input.color.blue + "}" + "]" + "}";

        result += "]";

        return "$" + result;
    }

    // shouldn't really need a decode

    public double getX() {
        return position.x;
    }
    public double getY() {
        return position.y;
    }
    public double getZ() {
        return position.z;
    }

    public double getDistance(nav_marker other) {
        double xDiff = other.position.x - position.x;
        double yDiff = other.position.y - position.y;
        double zDiff = other.position.z - position.z;

        return Math.sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);
    }
}