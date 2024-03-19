package frc.robot.commands.camera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.UserPolicy;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    PIDController xPIDController = new PIDController(0.015,0,0);
    //PIDController yPidController = new PIDController(VisionConstants.AUTO_ALIGN_P, VisionConstants.AUTO_ALIGN_I, VisionConstants.AUTO_ALIGN_D);
    public AutoAimCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        UserPolicy.canAutoAlign = false;
        //drive.drive(0, 0, 0, true, true);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasValidTarget()){
            double xPower = xPIDController.calculate(visionSubsystem.getTargetHorizontalOffset(), 0);
            xPIDController.setTolerance(0.015);
            DriveSubsystem.autoAimSpeed = xPower;

            //double yPower= yPidController.calculate(visionSubsystem.GetTargetVerticalOffset(), 0);
            //drive.drive(0, 0, -xPower, false, true);
        }
    }

    @Override
    public void initialize() {   
        UserPolicy.canAutoAlign = true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
