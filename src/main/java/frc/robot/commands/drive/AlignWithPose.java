package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotConstants.DriverAssistConstants.AutoBuilderConstants;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.CowboyUtils;
import frc.robot.RobotState;

public class AlignWithPose {

    public AlignWithPose() {

    }

    public static Command pathToPoseCommand(Pose2d target, DriveSubsystem driveSubsystem) {

        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(

                new PIDController(3, 0, 0),
                new PIDController(3, 0, 0), new ProfiledPIDController(3, 0, 0, new Constraints(3, 3)));
        holonomicDriveController.setTolerance(new Pose2d(1, 1, new Rotation2d(10)));
        Command roughAlignmentCommand = AutoBuilder.pathfindToPose(
                target,
                AutoBuilderConstants.PATH_FINDING_CONSTRAINTS,
                0.0,
                0);
        Command fineAlignmentCommand = new FunctionalCommand(
                () -> {
                },
                () -> {
                    ChassisSpeeds output = holonomicDriveController.calculate(RobotState.robotPose,
                            target, 1,
                            new Rotation2d(0));
                    output.div(DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
                    driveSubsystem.runChassisSpeeds(output, false);
                },
                (interrupted) -> {
                },
                () -> false);

        return new SequentialCommandGroup(roughAlignmentCommand, fineAlignmentCommand);
    }

    public static Command alignWithSpeakerCommand(DriveSubsystem driveSubsystem) {
        return pathToPoseCommand(CowboyUtils.getAllianceSpeaker(), driveSubsystem);
    }
}
