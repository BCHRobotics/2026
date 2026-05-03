package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/** No-op placeholder command for quick testing and experimentation. */
public class TestCommand extends Command {

    /** Creates a new TestCommand with no subsystem requirements. */
    public TestCommand() {}

    /** Called once when the command is first scheduled. */
    @Override
    public void initialize() {}

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {}

    /**
     * Returns true when the command should end.
     *
     * @return whether the command has finished
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Called once when the command ends or is interrupted.
     *
     * @param interrupted true if the command was interrupted before finishing
     */
    @Override
    public void end(boolean interrupted) {}
}
