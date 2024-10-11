package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.CowboyUtils;


public class AlignWithPose extends Command {
    DriveSubsystem driveSubsystem;
    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(

        new PIDController(3, 0, 0),
            new PIDController(3, 0, 0), new ProfiledPIDController(3, 0, 0, new Constraints(2, 2)));

    public AlignWithPose(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.drive(0, 0, 0, true, true);
    }

    @Override
    public void execute() {
        ChassisSpeeds output = holonomicDriveController.calculate(RobotState.robotPose,
                            CowboyUtils.getAllianceSpeaker(), .5,
                            new Rotation2d(0));
                    output.div(DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
                    driveSubsystem.runChassisSpeeds(output, false);

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
