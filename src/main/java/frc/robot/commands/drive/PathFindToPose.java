package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CowboyUtils;
import frc.robot.RobotConstants.PathFindToPoseConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PathFindToPose {
    DriveSubsystem driveSubsystem;
    public static Command pathfindingCommand;

    public static Command pathToPoseCommand(Pose2d target) {
        pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                PathFindToPoseConstants.PATH_FINDING_CONSTRAINTS,
                0.0,
                0.0);
        return pathfindingCommand;
    }

    public static Command alignWithSpeakerCommand() {
        return pathToPoseCommand(CowboyUtils.getAllainceSpeakerPose());
    }

}
