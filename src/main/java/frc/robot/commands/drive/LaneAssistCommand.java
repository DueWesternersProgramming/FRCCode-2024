package frc.robot.commands.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.RobotConstants.DriverAssistConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class LaneAssistCommand extends Command {
    Command command;

    public LaneAssistCommand() {
        addRequirements();
    }

    @Override
    public void initialize() {
        PathPlannerPath path = new PathPlannerPath(
                RobotContainer.getSelectedLane(), // This will be called every time the command is initialized
                new PathConstraints(DriverAssistConstants.MAX_VELOCITY, DriverAssistConstants.MAX_ACCELERATION,
                        DriverAssistConstants.MAX_ANGULAR_SPEED, DriverAssistConstants.MAX_ANGULAR_ACCELERATION),
                new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

        command = AutoBuilder.followPath(path);
        CommandScheduler.getInstance().schedule(command);
    }

    @Override
    public void execute() {
        System.out.println(command.isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        if (command != null) {
            command.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
