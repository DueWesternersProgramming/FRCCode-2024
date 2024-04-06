package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    private final ClimberSubsystem climber;
    private final Joystick operatorJoystick;
    
     public ClimberCommand(ClimberSubsystem climber, Joystick operatorJoystick) {
        this.climber = climber;
        this.operatorJoystick = operatorJoystick;
        addRequirements(climber);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        climber.setSpeed(operatorJoystick.getRawAxis(TeleopConstants.CLIMBER_LEFT_COMMAND_AXIS),
            operatorJoystick.getRawAxis(TeleopConstants.CLIMBER_RIGHT_COMMAND_AXIS), operatorJoystick.getRawButton(5));
    }
    
    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
