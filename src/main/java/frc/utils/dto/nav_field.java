package frc.utils.dto;

public class nav_field
{
    public nav_robot[] robots;
    public nav_path[] paths;
    public nav_marker[] markers;

    public String EncodeToString()
    {
        return nav_field.EncodeToString(this);
    }

    public static String EncodeToString(nav_field input)
    {
        // encoding looks like this:
        // "field(robots{[]}paths{[]}markers{[]})"

        String result = "";

        result += "field(";

        // *** robots ***
        result += "robots{";
        for (int i = 0; i < input.robots.length; i++)
        {
            result += nav_robot.EncodeToString(input.robots[i]);
        }
        result += "}";

        // *** paths ***
        result += "paths{";
        for (int i = 0; i < input.paths.length; i++)
        {
            result += nav_path.EncodeToString(input.paths[i]);
        }
        result += "}";

        // *** markers ***
        result += "markers{";
        for (int i = 0; i < input.markers.length; i++)
        {
            result += nav_marker.EncodeToString(input.markers[i]);
        }
        result += "}";

        result += ")";

        return "$" + result;
    }
}
