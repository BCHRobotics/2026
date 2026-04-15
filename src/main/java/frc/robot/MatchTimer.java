package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MatchTimer {
    // Match phase durations (seconds)
    private static final int D_AUTO = 20;
    private static final int D_TRANSITION = 10;
    private static final int D_SHIFT1 = 25;
    private static final int D_SHIFT2 = 25;
    private static final int D_SHIFT3 = 25;
    private static final int D_SHIFT4 = 25;
    private static final int D_ENDGAME = 30;

    private enum MatchPhase {
        NONE,
        AUTO,
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME
    }

    private final Timer timer = new Timer();
    private boolean running = false;
    // offset (seconds) to add to timer.get() so we can start at later phases
    private double offsetSeconds = 0.0;

    public MatchTimer() {
    }

    public void start() {
        // Start from the beginning (AUTO)
        offsetSeconds = 0.0;
        timer.reset();
        timer.start();
        running = true;
    }

    /**
     * Start the timer but treat the elapsed time as if we've already passed AUTO.
     * This begins the timer at the start of TRANSITION.
     */
    public void startTransition() {
        offsetSeconds = D_AUTO;
        timer.reset();
        timer.start();
        running = true;
    }

    public void stop() {
        timer.stop();
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    /**
     * Returns true when the current phase is one of the shift phases (SHIFT1..SHIFT4).
     */
    public boolean isInShiftPhase() {
        if (!running) {
            return false;
        }
        double elapsed = timer.get() + offsetSeconds;
        double cursor = 0.0;
        cursor += D_AUTO; // end of AUTO
        cursor += D_TRANSITION; // end of TRANSITION, start of shifts
        double startShifts = cursor;
        double endShifts = cursor + D_SHIFT1 + D_SHIFT2 + D_SHIFT3 + D_SHIFT4;
        return elapsed >= startShifts && elapsed < endShifts;
    }

    /**
     * Returns true when the current phase is TRANSITION.
     */
    public boolean isInTransitionPhase() {
        if (!running) {
            return false;
        }
        double elapsed = timer.get() + offsetSeconds;
        double cursor = 0.0;
        // AUTO
        cursor += D_AUTO;
        // TRANSITION
        double transitionStart = cursor;
        double transitionEnd = cursor + D_TRANSITION;
        return elapsed >= transitionStart && elapsed < transitionEnd;
    }

    /**
     * Update SmartDashboard entries using the current timer state and the
     * operator touchpad pressed state.
     *
     * @param touchpadPressed whether the operator touchpad is currently pressed
     */
    /**
     * Update SmartDashboard entries using the current timer state and the
     * latched shift toggle state.
     *
     * @param shiftToggle whether the operator has toggled the shift selection (latched)
     */
    public void updateDashboard(boolean shiftToggle) {
        int timeLeft = 0;
        MatchPhase phase = MatchPhase.NONE;

        if (running) {
            double elapsed = timer.get() + offsetSeconds;

            double cursor = 0.0;
            if (elapsed < (cursor += D_AUTO)) {
                phase = MatchPhase.AUTO;
                timeLeft = (int) Math.max(0, Math.ceil(cursor - elapsed));
            } else if (elapsed < (cursor += D_TRANSITION)) {
                phase = MatchPhase.TRANSITION;
                timeLeft = (int) Math.max(0, Math.ceil(cursor - elapsed));
            } else if (elapsed < (cursor += D_SHIFT1)) {
                phase = MatchPhase.SHIFT1;
                timeLeft = (int) Math.max(0, Math.ceil(cursor - elapsed));
            } else if (elapsed < (cursor += D_SHIFT2)) {
                phase = MatchPhase.SHIFT2;
                timeLeft = (int) Math.max(0, Math.ceil(cursor - elapsed));
            } else if (elapsed < (cursor += D_SHIFT3)) {
                phase = MatchPhase.SHIFT3;
                timeLeft = (int) Math.max(0, Math.ceil(cursor - elapsed));
            } else if (elapsed < (cursor += D_SHIFT4)) {
                phase = MatchPhase.SHIFT4;
                timeLeft = (int) Math.max(0, Math.ceil(cursor - elapsed));
            } else if (elapsed < (cursor += D_ENDGAME)) {
                phase = MatchPhase.ENDGAME;
                timeLeft = (int) Math.max(0, Math.ceil(cursor - elapsed));
            } else {
                phase = MatchPhase.NONE;
                timeLeft = 0;
                timer.stop();
                running = false;
            }
        }

        SmartDashboard.putNumber("Hub Timer", timeLeft);
        SmartDashboard.putString("Hub Phase", phase.name());

        boolean publishedBoolean = false;
        switch (phase) {
            case AUTO:
            case TRANSITION:
            case ENDGAME:
                publishedBoolean = true;
                break;
            case SHIFT1:
                // When shiftToggle==true => SHIFT1 true (pressed during transition selects 1&3)
                publishedBoolean = shiftToggle;
                break;
            case SHIFT2:
                // SHIFT2 is active when shiftToggle is false
                publishedBoolean = !shiftToggle;
                break;
            case SHIFT3:
                publishedBoolean = shiftToggle;
                break;
            case SHIFT4:
                publishedBoolean = !shiftToggle;
                break;
            default:
                publishedBoolean = false;
                break;
        }

        // Determine display state for the dashboard. If the hub will become active
        // within 5 seconds, flash the dashboard boolean at 2 Hz. Otherwise show
        // the actual publishedBoolean (solid true when active, false when not).
        boolean displayBoolean = publishedBoolean;
        if (!publishedBoolean) {
            double timeUntilNext = getTimeUntilNextActive(shiftToggle);
            if (timeUntilNext > 0 && timeUntilNext <= 5.0) {
                boolean flashState = ((long) (timer.get() * 4)) % 2 == 0;
                displayBoolean = flashState;
            }
        }

        SmartDashboard.putBoolean("Hub Active", displayBoolean);
    }

    /**
     * Returns seconds until the next phase where Hub Active would be true, using
     * the provided shiftToggle mapping. Returns Double.POSITIVE_INFINITY if no
     * future active phase exists.
     */
    private double getTimeUntilNextActive(boolean shiftToggle) {
        if (!running) {
            return Double.POSITIVE_INFINITY;
        }
        double elapsed = timer.get() + offsetSeconds;

        // Build start times for phases
        double startAuto = 0.0;
        double startTransition = startAuto + D_AUTO;
        double startShift1 = startTransition + D_TRANSITION;
        double startShift2 = startShift1 + D_SHIFT1;
        double startShift3 = startShift2 + D_SHIFT2;
        double startShift4 = startShift3 + D_SHIFT3;
        double startEndgame = startShift4 + D_SHIFT4;

        // Helper to compute publishedBoolean for a phase
        java.util.function.BiFunction<MatchPhase, Boolean, Boolean> pubFor = (ph, st) -> {
            switch (ph) {
                case AUTO:
                case TRANSITION:
                case ENDGAME:
                    return true;
                case SHIFT1:
                    return st;
                case SHIFT2:
                    return !st;
                case SHIFT3:
                    return st;
                case SHIFT4:
                    return !st;
                default:
                    return false;
            }
        };

        // Check phases in order starting at current elapsed
        if (elapsed <= startAuto) {
            if (pubFor.apply(MatchPhase.AUTO, shiftToggle)) return startAuto - elapsed;
            if (pubFor.apply(MatchPhase.TRANSITION, shiftToggle)) return startTransition - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT1, shiftToggle)) return startShift1 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT2, shiftToggle)) return startShift2 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT3, shiftToggle)) return startShift3 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT4, shiftToggle)) return startShift4 - elapsed;
            if (pubFor.apply(MatchPhase.ENDGAME, shiftToggle)) return startEndgame - elapsed;
            return Double.POSITIVE_INFINITY;
        } else if (elapsed <= startTransition) {
            if (pubFor.apply(MatchPhase.TRANSITION, shiftToggle)) return startTransition - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT1, shiftToggle)) return startShift1 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT2, shiftToggle)) return startShift2 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT3, shiftToggle)) return startShift3 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT4, shiftToggle)) return startShift4 - elapsed;
            if (pubFor.apply(MatchPhase.ENDGAME, shiftToggle)) return startEndgame - elapsed;
            return Double.POSITIVE_INFINITY;
        } else if (elapsed <= startShift1) {
            if (pubFor.apply(MatchPhase.SHIFT1, shiftToggle)) return startShift1 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT2, shiftToggle)) return startShift2 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT3, shiftToggle)) return startShift3 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT4, shiftToggle)) return startShift4 - elapsed;
            if (pubFor.apply(MatchPhase.ENDGAME, shiftToggle)) return startEndgame - elapsed;
            return Double.POSITIVE_INFINITY;
        } else if (elapsed <= startShift2) {
            if (pubFor.apply(MatchPhase.SHIFT2, shiftToggle)) return startShift2 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT3, shiftToggle)) return startShift3 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT4, shiftToggle)) return startShift4 - elapsed;
            if (pubFor.apply(MatchPhase.ENDGAME, shiftToggle)) return startEndgame - elapsed;
            return Double.POSITIVE_INFINITY;
        } else if (elapsed <= startShift3) {
            if (pubFor.apply(MatchPhase.SHIFT3, shiftToggle)) return startShift3 - elapsed;
            if (pubFor.apply(MatchPhase.SHIFT4, shiftToggle)) return startShift4 - elapsed;
            if (pubFor.apply(MatchPhase.ENDGAME, shiftToggle)) return startEndgame - elapsed;
            return Double.POSITIVE_INFINITY;
        } else if (elapsed <= startShift4) {
            if (pubFor.apply(MatchPhase.SHIFT4, shiftToggle)) return startShift4 - elapsed;
            if (pubFor.apply(MatchPhase.ENDGAME, shiftToggle)) return startEndgame - elapsed;
            return Double.POSITIVE_INFINITY;
        } else if (elapsed <= startEndgame) {
            if (pubFor.apply(MatchPhase.ENDGAME, shiftToggle)) return startEndgame - elapsed;
            return Double.POSITIVE_INFINITY;
        }

        return Double.POSITIVE_INFINITY;
    }
}
