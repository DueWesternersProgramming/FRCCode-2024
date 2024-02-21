package frc.robot.commands.drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.AutonomousConstants;
import frc.robot.subsystems.DriveSubsystem;


public class SnapToHeadingCommand extends Command {
    private final DriveSubsystem drive;
    private double m_angle;
    private PIDController rotPIDController = new PIDController(AutonomousConstants.THETA_CONTROLLER_P, AutonomousConstants.THETA_CONTROLLER_I, AutonomousConstants.THETA_CONTROLLER_D);
    
    public SnapToHeadingCommand(DriveSubsystem drive, double angle) {
        this.drive = drive;
        m_angle = angle;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, DrivetrainConstants.kFieldRelative, true);
    }

    @Override
    public void execute() {
        
            double rotPower = rotPIDController.calculate(drive.getHeadingDegrees(), m_angle);
            
            rotPIDController.setTolerance(2);
            //double yPower= yPidController.calculate(visionSubsystem.GetTargetVerticalOffset(), 0);
            drive.drive(0, 0, -rotPower*0.1, false, true);
        }

    @Override
    public void initialize() {   
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
