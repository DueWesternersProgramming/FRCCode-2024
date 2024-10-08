package frc.robot.commands.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CowboyUtils;
import frc.robot.RobotContainer;

import frc.robot.RobotConstants.DriverAssistConstants;

public class LaneAssist {
    private static Command onTheFlyLaneAssistCommand(List<Translation2d> waypoints) {

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                new PathConstraints(DriverAssistConstants.MAX_VELOCITY, DriverAssistConstants.MAX_ACCELERATION,
                        DriverAssistConstants.MAX_ANGULAR_SPEED, DriverAssistConstants.MAX_ANGULAR_ACCELERATION),
                new GoalEndState(0.0, Rotation2d.fromDegrees(0)));
        return AutoBuilder.followPath(path);

    }

    public static Command laneAssistCommand() {
        return onTheFlyLaneAssistCommand(RobotContainer.getSelectedLane());
    }

}
