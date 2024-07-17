package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CowboyUtils;
import frc.robot.RobotConstants.PathFindToPoseConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

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
        if (DriverStation.getAlliance().isPresent())
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                return pathToPoseCommand(CowboyUtils.blueAllianceSpeaker);
            } else {
                return pathToPoseCommand(CowboyUtils.redAllianceSpeaker);
            }
        return pathToPoseCommand(CowboyUtils.blueAllianceSpeaker);
    }

}
