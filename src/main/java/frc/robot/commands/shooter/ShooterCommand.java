package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final Joystick operatorJoystick;
    
     public ShooterCommand(ShooterSubsystem shooter, Joystick operatorJoystick) {
        this.shooter = shooter;
        this.operatorJoystick = operatorJoystick;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        shooter.setShooterSpeed(operatorJoystick.getRawAxis(TeleopConstants.SHOOTER_COMMAND_AXIS));
    }
    
    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
