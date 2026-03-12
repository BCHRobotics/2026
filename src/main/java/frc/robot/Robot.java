// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class and to call the
 * functions corresponding to each mode as described in the TimedRobot
 * documentation.
 *
 * <p><b>LED management:</b> all LED logic is centralised in
 * {@link RobotContainer#updateLEDState()}, called once per {@link #robotPeriodic()}
 * so it runs in every mode — disabled, auto, teleop, and test — with a single
 * priority-ordered chain.  No LED code belongs in the mode-specific callbacks.
 */
public class Robot extends TimedRobot {

    private Command        m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /** Tracks the current alliance for PathPlanner path mirroring. */
    public static boolean isRed;

    // ── Init ─────────────────────────────────────────────────────────────────

    /**
     * Called once when robot code first starts.
     *
     * <p>Constructing {@link RobotContainer} here constructs every subsystem,
     * including {@code LED}, which starts its 2-second deploy timer at this exact
     * moment.  {@code updateLEDState()} will therefore show solid white (DEPLOY)
     * for the first 2 seconds of every new code deploy.
     */
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    // ── Always-running ────────────────────────────────────────────────────────

    /**
     * Called every 20 ms regardless of mode.
     *
     * <p>Runs the {@link CommandScheduler} (which polls buttons, advances commands,
     * and calls all subsystem {@code periodic()} methods), then updates the LED
     * strip through the centralised priority chain.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.updateLEDState();
    }

    // ── Disabled ─────────────────────────────────────────────────────────────

    @Override
    public void disabledInit() {
        // updateLEDState() handles the DISABLED state automatically every loop.
    }

    @Override
    public void disabledPeriodic() {}

    // ── Autonomous ────────────────────────────────────────────────────────────

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    // ── Teleop ────────────────────────────────────────────────────────────────

    @Override
    public void teleopInit() {
        // Switch to LocalADStar for on-the-fly pathfinding during teleop.
        Pathfinding.setPathfinder(new LocalADStar());

        // Cancel any auto routine still running from the autonomous period.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    // ── Test ─────────────────────────────────────────────────────────────────

    @Override
    public void testInit() {
        // Cancel everything before entering test mode.
        // updateLEDState() will switch to solid purple (TEST) automatically.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}