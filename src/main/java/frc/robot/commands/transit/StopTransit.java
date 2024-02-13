package frc.robot.commands.transit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitSubsystem;

public class StopTransit extends Command {
    private final TransitSubsystem transit;
    
     public StopTransit(TransitSubsystem transit) {
        this.transit = transit;
        addRequirements(transit);
    }

    @Override
    public void initialize() {
        transit.transitoff();
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
