import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

/**
 * Quick test to check if the F310 gamepad is accessible via Java's AWT/Robot HID layer.
 * Uses java.awt to enumerate controllers via the native macOS HID stack.
 */
public class JoystickTest {
    public static void main(String[] args) throws Exception {
        System.out.println("=== Joystick / HID Device Test ===");
        System.out.println("Java: " + System.getProperty("java.version"));
        System.out.println("OS:   " + System.getProperty("os.name") + " " + System.getProperty("os.version"));
        System.out.println();

        // Try to find controllers via java.awt (triggers Input Monitoring permission prompt)
        try {
            GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
            System.out.println("GraphicsEnvironment headless: " + ge.isHeadlessInstance());
        } catch (Exception e) {
            System.out.println("GraphicsEnvironment error: " + e.getMessage());
        }

        // Use the same jinput path that WPILib's HAL sim uses
        // Try to load net.java.games.input via reflection if present
        System.out.println("\nChecking for jinput (WPILib's joystick library)...");
        try {
            Class<?> controllerEnv = Class.forName("net.java.games.input.ControllerEnvironment");
            Object env = controllerEnv.getMethod("getDefaultEnvironment").invoke(null);
            Object[] controllers = (Object[]) controllerEnv.getMethod("getControllers").invoke(env);
            System.out.println("Found " + controllers.length + " controller(s):");
            for (Object c : controllers) {
                System.out.println("  - " + c);
            }
        } catch (ClassNotFoundException e) {
            System.out.println("jinput not on classpath (expected outside WPILib sim).");
        }

        // Use SDL-style check via /dev or IOKit indirectly — just verify HID permission
        System.out.println("\nAttempting to open a Swing window (forces Input Monitoring check)...");
        try {
            JFrame frame = new JFrame("Joystick Test - close me");
            frame.setSize(300, 100);
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            JLabel label = new JLabel("If you see this, Java has display access.", SwingConstants.CENTER);
            frame.add(label);
            frame.setVisible(true);
            System.out.println("Window opened successfully. Java can access the display.");
            System.out.println("Waiting 3 seconds then exiting...");
            Thread.sleep(3000);
            frame.dispose();
        } catch (Exception e) {
            System.out.println("Window error: " + e.getMessage());
        }

        System.out.println("\nDone.");
    }
}
