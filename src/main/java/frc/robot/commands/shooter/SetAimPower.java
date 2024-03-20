package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SetAimPower extends Command {
    private double power;
    private Joystick operatorJoystick;
    private final ShooterSubsystem shooter;
    
     public SetAimPower(ShooterSubsystem shooter, Joystick operatorJoystick) {
        this.shooter = shooter;
        this.operatorJoystick = operatorJoystick;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterAim(operatorJoystick.getRawAxis(0)*0.1);
    }
    
    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
