package frc.robot.commands.camera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.UserPolicy;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    PIDController xPIDController = new PIDController(0.015,0,0);
    public AutoAimCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        UserPolicy.canAutoAlign = false;
        DriveSubsystem.autoAimSpeed = 0;
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasResults()){
            double xPower = xPIDController.calculate(visionSubsystem.getTargetYaw(), 0);
            xPIDController.setTolerance(0.015);
            DriveSubsystem.autoAimSpeed = xPower;
        }
        else {
            DriveSubsystem.autoAimSpeed = 0;
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
