package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * Subsystem managing a non-addressable 12V RGB LED strip.
 *
 * <p>State priority (highest to lowest):
 * <ol>
 *   <li>POSE_VALID – flashing lime, AprilTag pose lock active (teleop)</li>
 *   <li>AUTO      – solid orange, autonomous period running</li>
 *   <li>TELEOP    – solid green, teleop enabled</li>
 *   <li>DISABLED  – solid red, robot disabled or e-stopped</li>
 * </ol>
 *
 * Hardware: 12V RGB strip driven by three PWM-controlled MOSFET channels.
 * Each color channel is controlled independently using the DIO/PWM channels
 * defined in {@link LEDConstants}.
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

    private final DigitalOutput m_redChannel;
    private final DigitalOutput m_greenChannel;
    private final DigitalOutput m_blueChannel;

    private State m_state = State.DISABLED;

    private final Timer m_flashTimer = new Timer();
    private boolean m_flashOn = true;

    public LED() {
        m_redChannel = new DigitalOutput(LEDConstants.kRedChannel);
        m_greenChannel = new DigitalOutput(LEDConstants.kGreenChannel);
        m_blueChannel = new DigitalOutput(LEDConstants.kBlueChannel);

        m_redChannel.setPWMRate(LEDConstants.kPwmRateHz);
        m_greenChannel.setPWMRate(LEDConstants.kPwmRateHz);
        m_blueChannel.setPWMRate(LEDConstants.kPwmRateHz);

        m_redChannel.enablePWM(0.0);
        m_greenChannel.enablePWM(0.0);
        m_blueChannel.enablePWM(0.0);

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

    }

    private void setSolid(int[] rgb) {
        m_redChannel.updateDutyCycle(toDutyCycle(rgb[0]));
        m_greenChannel.updateDutyCycle(toDutyCycle(rgb[1]));
        m_blueChannel.updateDutyCycle(toDutyCycle(rgb[2]));
    }

    private double toDutyCycle(int value) {
        return Math.max(0.0, Math.min(1.0, value / 255.0));
    }
}
