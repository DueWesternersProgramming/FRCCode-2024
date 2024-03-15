package frc.robot.commands.drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;


public class SnapToHeadingCommand extends Command {
    private final DriveSubsystem drive;
    private double m_angle;
    private PIDController rotPIDController = new PIDController(0.009,0,0.25);
    
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
        
            double rotPower = rotPIDController.calculate(drive.getHeading().get(), m_angle);
            //System.out.println(drive.getHeading());
            
            drive.drive(0, 0, -rotPower, false, true);
        }

    @Override
    public void initialize() {   
        rotPIDController.setTolerance(0.05);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
