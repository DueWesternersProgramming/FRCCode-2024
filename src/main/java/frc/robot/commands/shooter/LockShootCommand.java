package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.UserPolicy;


public class LockShootCommand extends Command {

    private final boolean locked;

    public LockShootCommand(boolean locked) {
        this.locked = locked;
    }

    @Override
    public void initialize() {
        UserPolicy.shootCommandLocked = locked;

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
