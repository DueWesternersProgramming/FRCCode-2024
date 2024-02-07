package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimCommand extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem visionSubsystem;
    PIDController xPIDController = new PIDController(VisionConstants.AUTO_ALIGN_P, VisionConstants.AUTO_ALIGN_I, VisionConstants.AUTO_ALIGN_D);
    PIDController yPidController = new PIDController(VisionConstants.AUTO_ALIGN_P, VisionConstants.AUTO_ALIGN_I, VisionConstants.AUTO_ALIGN_D);
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
        if (visionSubsystem.HasValidTarget()){
            double xPower = xPIDController.calculate(visionSubsystem.GetTargetHorizontalOffset(), 0);
            double yPower= yPidController.calculate(visionSubsystem.GetTargetVerticalOffset(), 0);
            drive.drive(0, xPower, 0, false, true);
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
