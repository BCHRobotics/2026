package frc.utils;

import com.pathplanner.lib.config.RobotConfig;

public class AutoUtils {
    public static RobotConfig getRobotConfig() {
        RobotConfig config = null;
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

        return config;
    }
}
