package frc.robot.commands.drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class SnapToHeadingCommand extends Command {
    private final DriveSubsystem drive;
    private double m_angle;
    private PIDController rotPIDController = new PIDController(0.009,0,0);
    
    public SnapToHeadingCommand(DriveSubsystem drive, double angle) {
        this.drive = drive;
        m_angle = angle;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {   
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
