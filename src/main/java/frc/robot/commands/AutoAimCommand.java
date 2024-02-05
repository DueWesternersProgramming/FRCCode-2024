package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimCommand extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem visionSubsystem;
    PIDController aimPidController = new PIDController(1, 0, 0);   //TODO: ADD CONSTANTS FOR THIS


    public AutoAimCommand(DriveSubsystem drive, VisionSubsystem visionSubsystem) {
        this.drive = drive;
        this.visionSubsystem = visionSubsystem;
        addRequirements(drive,visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, DrivetrainConstants.kFieldRelative, true);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasResults()){
            double power = aimPidController.calculate(visionSubsystem.getTargetYaw(), 0);
            drive.drive(0, 0, power, false, true);

        }
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
