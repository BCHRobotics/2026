package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * Subsystem managing the on-robot WS2812B addressable LED strip.
 *
 * <p>All states use solid colors only — no per-LED animations or sweeps.
 * "Flashing" means the whole strip alternates between two solid colors at 4 Hz.
 *
 * <p><b>State priority (highest → lowest):</b>
 * <ol>
 *   <li>{@code DEPLOY}          – solid white, 2 s immediately after code boots</li>
 *   <li>{@code GYRO_FAULT}      – flashing red-orange ↔ red, NavX disconnected</li>
 *   <li>{@code DISABLED}        – solid red, robot disabled or e-stopped</li>
 *   <li>{@code AUTO}            – solid orange, autonomous period active</li>
 *   <li>{@code TEST}            – solid purple, Driver Station in test mode</li>
 *   <li>{@code MATCH_END}       – flashing white ↔ off, ≤ 10 s remaining in teleop</li>
 *   <li>{@code ENDGAME}         – flashing yellow ↔ off, ≤ 30 s remaining in teleop</li>
 *   <li>{@code AUTO_ASSIST}     – solid blue, a navigation/assist command is driving</li>
 *   <li>{@code POSE_VALID}      – distance gradient (red→yellow→green) or flashing lime</li>
 *   <li>{@code INTAKE}          – solid orange, intake motor is running</li>
 *   <li>{@code SHIFT_ON}        – solid cyan, shooting window is open (25 s cycle)</li>
 *   <li>{@code SHIFT_OFF}       – solid dark-blue, shooting window is closed</li>
 *   <li>{@code CLIMBER}         – solid teal (placeholder — wire up when climber added)</li>
 *   <li>{@code MECHANISM_READY} – solid lime (placeholder — wire up when scorer added)</li>
 *   <li>{@code TELEOP}          – solid green, default — driver in full control</li>
 * </ol>
 *
 * <p><b>Distance indicator:</b> call {@link #setDistanceToTarget(double)} every loop
 * while vision is active.  When state is {@code POSE_VALID} the whole strip shows one
 * solid color that shifts red → yellow → green as the robot closes in.  Pass {@code -1}
 * to fall back to flashing lime.
 *
 * <p><b>Hardware:</b> single WS2812B-compatible strip wired to the PWM port defined by
 * {@link LEDConstants#kPort}, containing {@link LEDConstants#kLength} LEDs.
 */
public class LED extends SubsystemBase {

    // ── State enum ────────────────────────────────────────────────────────────
    public enum State {
        DEPLOY,
        GYRO_FAULT,
        DISABLED,
        AUTO,
        TEST,
        MATCH_END,
        ENDGAME,
        AUTO_ASSIST,
        POSE_VALID,
        INTAKE,
        SHIFT_ON,
        SHIFT_OFF,
        SHIFT_ON_WARNING,  
        SHIFT_OFF_WARNING,
        CLIMBER,
        MECHANISM_READY,
        TELEOP
    }

    // ── Colors [R, G, B] ──────────────────────────────────────────────────────
    private static final int[] WHITE      = {255, 255, 255};  // DEPLOY
    private static final int[] GREEN      = {  0, 200,   0};  // TELEOP
    private static final int[] ORANGE     = {255,  80,   0};  // AUTO, INTAKE
    private static final int[] RED        = {255,   0,   0};  // DISABLED
    private static final int[] LIME       = {130, 255,   0};  // POSE_VALID fallback (flashing) + MECHANISM_READY (solid)
    private static final int[] YELLOW     = {255, 180,   0};  // ENDGAME (flashing)
    private static final int[] CYAN       = {  0, 220, 220};  // SHIFT_ON
    private static final int[] DARK_BLUE  = {  0,  30, 160};  // SHIFT_OFF
    private static final int[] BLUE       = {  0,  80, 255};  // AUTO_ASSIST
    private static final int[] PURPLE     = {160,   0, 200};  // TEST
    private static final int[] TEAL       = {  0, 180, 130};  // CLIMBER
    private static final int[] RED_ORANGE = {255,  50,   0};  // GYRO_FAULT (flashing alt)
    private static final int[] OFF        = {  0,   0,   0};  // Flash "dark" half

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final AddressableLED       m_led;
    private final AddressableLEDBuffer m_buffer;

    // ── State ─────────────────────────────────────────────────────────────────
    /** Currently active LED state — written every loop by RobotContainer.updateLEDState(). */
    private State m_state = State.DISABLED;

    /**
     * Distance to the nearest AprilTag in metres.
     * {@code -1} = no data; POSE_VALID falls back to flashing lime.
     */
    private double m_distanceMeters = -1.0;

    // ── Timers ────────────────────────────────────────────────────────────────
    /** Toggles {@code m_flashOn} at {@link LEDConstants#kFlashPeriod} Hz. */
    private final Timer m_flashTimer = new Timer();
    /** True during the "lit" half of any flashing state. */
    private boolean m_flashOn = true;

    /** Tracks elapsed time since construction for the 2-second deploy window. */
    private final Timer m_deployTimer = new Timer();
    /** How long (seconds) to show solid white after a new code deploy. */
    private static final double kDeployDuration = 2.0;
    /** Latched permanently to true once the deploy window closes. */
    private boolean m_deployPlayed = false;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Creates the LED subsystem and starts both internal timers.
     *
     * <p>Constructed inside {@link frc.robot.RobotContainer}, which is itself
     * constructed in {@link frc.robot.Robot#robotInit()}.  The deploy timer therefore
     * starts the instant new code boots on the RoboRIO — exactly when the 2-second
     * deploy confirmation should begin.
     */
    public LED() {
        m_led    = new AddressableLED(LEDConstants.kPort);
        m_buffer = new AddressableLEDBuffer(LEDConstants.kLength);
        m_led.setLength(LEDConstants.kLength);
        m_led.start();

        m_flashTimer.start();
        m_deployTimer.start();
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Sets the desired LED state.  Call once per loop from
     * {@code RobotContainer.updateLEDState()}.
     *
     * @param state the state to display
     */
    public void setState(State state) {
        m_state = state;
    }

    /**
     * Supplies the distance to the nearest visible AprilTag for the
     * {@code POSE_VALID} distance-gradient mode.
     *
     * <p>The whole strip shows one solid color that moves red → yellow → green
     * as the robot approaches.  Pass {@code -1} to fall back to flashing lime.
     *
     * @param meters distance in metres, or {@code -1} if unknown
     */
    public void setDistanceToTarget(double meters) {
        m_distanceMeters = meters;
    }

    /**
     * Returns {@code true} for the first {@value #kDeployDuration} seconds after
     * construction — i.e. the first 2 s of every new code deploy.
     *
     * <p>Used by {@code RobotContainer.updateLEDState()} as the highest-priority
     * check so the deploy white overrides every other state at boot time.
     *
     * @return {@code true} while the deploy window is still active
     */
    public boolean isDeployAnimationPlaying() {
        return !m_deployPlayed && m_deployTimer.get() < kDeployDuration;
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {

        // Advance the flash toggle — fires once per kFlashPeriod and auto-resets.
        if (m_flashTimer.advanceIfElapsed(LEDConstants.kFlashPeriod)) {
            m_flashOn = !m_flashOn;
        }

        // Latch the deploy flag once the 2-second window closes.
        // Without this latch, isDeployAnimationPlaying() would stay false but we
        // still want m_deployPlayed set so the timer can be garbage-collected cheaply.
        if (!m_deployPlayed && m_deployTimer.get() >= kDeployDuration) {
            m_deployPlayed = true;
        }

        // ── Render ────────────────────────────────────────────────────────────
        switch (m_state) {

            case DEPLOY:
                // Solid white: new code is live. Tells pit crew deploy succeeded.
                setSolid(WHITE);
                break;

            case GYRO_FAULT:
                // Flashing red-orange ↔ red at 4 Hz.
                // NavX disconnected — field-relative steering is broken.
                setSolid(m_flashOn ? RED_ORANGE : RED);
                break;

            case DISABLED:
                // Solid red — robot is disabled or e-stopped.
                setSolid(RED);
                break;

            case AUTO:
                // Solid orange — autonomous period is running.
                setSolid(ORANGE);
                break;

            case TEST:
                // Solid purple — Driver Station is in test mode.
                setSolid(PURPLE);
                break;

            case MATCH_END:
                // Flashing white ↔ off at 4 Hz — ≤ 10 s left. Most urgent timing cue.
                setSolid(m_flashOn ? WHITE : OFF);
                break;

            case ENDGAME:
                // Flashing yellow ↔ off at 4 Hz — ≤ 30 s left. Start climbing.
                setSolid(m_flashOn ? YELLOW : OFF);
                break;

            case AUTO_ASSIST:
                // Solid blue — a navigation/assist command is steering the robot.
                // Reverts the instant the command ends and TeleopDrive resumes.
                setSolid(BLUE);
                break;

            case POSE_VALID:
                // Vision is locked on at least one valid AprilTag this cycle.
                // With distance data:    gradient color (red → yellow → green).
                // Without distance data: flashing lime fallback.
                if (m_distanceMeters >= 0) {
                    setSolid(distanceToColor(m_distanceMeters));
                } else {
                    setSolid(m_flashOn ? LIME : OFF);
                }
                break;

            case INTAKE:
                // Solid orange — intake motor is commanded above the deadband.
                // Orange is shared with AUTO but the two states are mutually exclusive:
                // INTAKE only fires during teleop (after all mode checks pass).
                setSolid(ORANGE);
                break;
            case SHIFT_ON_WARNING:
                // Flashing cyan ↔ off — shift is about to open. Get ready to shoot.
                setSolid(m_flashOn ? CYAN : OFF);
                break;

            case SHIFT_OFF_WARNING:
                // Flashing dark-blue ↔ off — shift is about to close. Stop shooting.
                setSolid(m_flashOn ? DARK_BLUE : OFF);
                break;
            case SHIFT_ON:
                // Solid cyan — the current 25-second window is the shooting window.
                setSolid(CYAN);
                break;

            case SHIFT_OFF:
                // Solid dark-blue — the current 25-second window is NOT the shooting window.
                setSolid(DARK_BLUE);
                break;

            case CLIMBER:
                // Solid teal — placeholder until the climber subsystem is wired up.
                setSolid(TEAL);
                break;

            case MECHANISM_READY:
                // Solid lime — placeholder until the scoring mechanism is wired up.
                // LIME is also used (flashing) as the POSE_VALID fallback — the two are
                // visually distinct: solid lime here vs. blinking lime/off in POSE_VALID.
                setSolid(LIME);
                break;

            case TELEOP:
            default:
                // Solid green — driver in full control, no special condition active.
                setSolid(GREEN);
                break;
        }

        // Push the buffer to the physical LED strip every loop.
        m_led.setData(m_buffer);

        // Publish the current state name so drivers / programmers can see it live.
        SmartDashboard.putString("LED State", m_state.toString());
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Sets every LED on the strip to the same solid color.
     *
     * @param rgb int[3]: {red, green, blue}, each value 0–255
     */
    private void setSolid(int[] rgb) {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
    }

    /**
     * Maps a distance value to one solid color on the red → yellow → green spectrum.
     * The entire strip shows that single color — this is NOT a per-LED gradient.
     *
     * <p>Two-segment interpolation avoids the muddy brown that a naive linear RGB
     * ramp would produce:
     * <ul>
     *   <li>Segment 1 (far  → mid):   red   {255, 0, 0}   → yellow {255, 180, 0}</li>
     *   <li>Segment 2 (mid  → close): yellow {255, 180, 0} → green  {0, 255, 0}</li>
     * </ul>
     *
     * @param distanceMeters current distance to nearest AprilTag in metres
     * @return int[3] RGB array for the mapped color
     */
    private int[] distanceToColor(double distanceMeters) {
        // t = 0.0 at kDistanceFarMeters (full red), t = 1.0 at kDistanceCloseMeters (full green).
        double range = LEDConstants.kDistanceFarMeters - LEDConstants.kDistanceCloseMeters;
        double t = 1.0 - (distanceMeters - LEDConstants.kDistanceCloseMeters) / range;
        t = Math.max(0.0, Math.min(1.0, t)); // clamp to [0, 1]

        int r, g;
        if (t < 0.5) {
            // Segment 1: red → yellow  (t: 0.0 → 0.5)
            double s = t * 2.0;             // re-scale to [0, 1]
            r = 255;
            g = (int)(180.0 * s);           // green rises 0 → 180
        } else {
            // Segment 2: yellow → green  (t: 0.5 → 1.0)
            double s = (t - 0.5) * 2.0;    // re-scale to [0, 1]
            r = (int)(255.0 * (1.0 - s));  // red falls  255 → 0
            g = (int)(180.0 + 75.0 * s);   // green rises 180 → 255
        }

        return new int[]{r, g, 0}; // no blue in this spectrum
    }
}