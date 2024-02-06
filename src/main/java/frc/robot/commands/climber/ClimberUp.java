package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUp extends Command {
    private final ClimberSubsystem climber;
    private boolean finished;
    
     public ClimberUp(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
       finished = false;
    }
    
    @Override
    public void execute() {
        if (climber.getEncoder1Position() < ClimberConstants.CLIMBER_UP_POSITION){
            climber.setSpeed(ClimberConstants.CLIMBER_MOTOR_SPEED);
        }
        else {
            finished = true;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        climber.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
