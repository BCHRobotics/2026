package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * Subsystem managing the on-robot RGB LED strip.
 *
 * <p>State priority (highest to lowest):
 * <ol>
 *   <li>POSE_VALID – flashing lime, AprilTag pose lock active (teleop)</li>
 *   <li>AUTO      – solid orange, autonomous period running</li>
 *   <li>TELEOP    – solid green, teleop enabled</li>
 *   <li>DISABLED  – solid red, robot disabled or e-stopped</li>
 * </ol>
 *
 * Hardware: single WS2812B-compatible addressable LED strip wired to the
 * PWM port defined by {@link LEDConstants#kPort}.
 */
public class LED extends SubsystemBase {

    public enum State {
        DISABLED,
        AUTO,
        TELEOP,
        POSE_VALID
    }

    // ---- Colors (R, G, B) ----
    private static final int[] GREEN   = {0,   200, 0};
    private static final int[] ORANGE  = {255, 80,  0};
    private static final int[] RED     = {255, 0,   0};
    private static final int[] LIME    = {130, 255, 0};
    private static final int[] OFF     = {0,   0,   0};

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    private State m_state = State.DISABLED;

    private final Timer m_flashTimer = new Timer();
    private boolean m_flashOn = true;

    public LED() {
        m_led = new AddressableLED(LEDConstants.kPort);
        m_buffer = new AddressableLEDBuffer(LEDConstants.kLength);
        m_led.setLength(LEDConstants.kLength);
        m_led.start();
        m_flashTimer.start();
    }

    /** Sets the current LED state. Call from {@code Robot} lifecycle methods. */
    public void setState(State state) {
        m_state = state;
    }

    @Override
    public void periodic() {
        // Toggle flash flag at the configured interval
        if (m_flashTimer.advanceIfElapsed(LEDConstants.kFlashPeriod)) {
            m_flashOn = !m_flashOn;
        }

        switch (m_state) {
            case POSE_VALID:
                setSolid(m_flashOn ? LIME : OFF);
                break;
            case AUTO:
                setSolid(ORANGE);
                break;
            case TELEOP:
                setSolid(GREEN);
                break;
            case DISABLED:
            default:
                setSolid(RED);
                break;
        }

        m_led.setData(m_buffer);
    }

    private void setSolid(int[] rgb) {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
    }
}
