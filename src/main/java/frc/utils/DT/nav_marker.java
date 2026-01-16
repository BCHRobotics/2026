package frc.utils.DT;

import frc.utils.M.Vector3;

/*
 * trying to make sure I don't have any duplicate classes here
 */

public class nav_marker {
    public Vector3 position;
    public double zAngle;

    public nav_marker() {}

    public nav_marker(Vector3 position, double zAngle) {
        this.position = position;
        this.zAngle = zAngle;
    }

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