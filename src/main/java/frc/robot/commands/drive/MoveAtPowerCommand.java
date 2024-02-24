package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotConstants.SubsystemEnabledConstants;

public class MoveAtPowerCommand extends Command {
    private final DriveSubsystem drive;
    private double x;
    private double y;
    private double rot;

    public MoveAtPowerCommand(DriveSubsystem drive, double x, double y, double rot) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, DrivetrainConstants.kFieldRelative, true);
    }

    @Override
    public void execute() {

        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED){
            drive.drive(-y, -x, -rot, false, true);
        } else {
            drive.drive(0, 0, 0, false, true);
        }
        
    }

    @Override
    public void initialize() {
        drive.drive(0, 0, 0, DrivetrainConstants.kFieldRelative, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
