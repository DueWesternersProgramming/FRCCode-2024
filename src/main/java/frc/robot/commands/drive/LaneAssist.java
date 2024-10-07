package frc.robot.commands.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants.FieldPointPoses;

public class LaneAssist {
    private static Command onTheFlyLaneAssistCommand(List<Translation2d> waypoints) {

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
                new GoalEndState(0.0, Rotation2d.fromDegrees(0)));
        return AutoBuilder.followPath(path);

    }

    public static Command laneAssistCommand() {
        return onTheFlyLaneAssistCommand(FieldPointPoses.BlueAlliance.MIDDLE_LANE_WAYPOINTS);
    }

}
