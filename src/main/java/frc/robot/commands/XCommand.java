package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.UserPolicy;

public class XCommand extends Command {
    public XCommand() {

    }

    @Override
    public void initialize() {
        UserPolicy.xLocked = !UserPolicy.xLocked;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
