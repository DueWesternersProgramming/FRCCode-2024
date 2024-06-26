package frc.robot.commands.transit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.TransitConstants;
import frc.robot.subsystems.TransitSubsystem;

public class ReverseTransit extends Command {
    private final TransitSubsystem transit;
    
     public ReverseTransit(TransitSubsystem transit) {
        this.transit = transit;
        addRequirements(transit);
    }

    @Override
    public void initialize() {
        transit.SetTransitSpeed(TransitConstants.REVERSE_TRANSIT_MOTOR_SPEED);
    }
    
    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
