package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.UserPolicy;

public class TwistCommand extends Command {
    public TwistCommand() {

    }

    @Override
    public void initialize() {
        UserPolicy.twistable = true;
    }

    @Override
    public void end(boolean interrupted) {
        UserPolicy.twistable = false;
    }
}
