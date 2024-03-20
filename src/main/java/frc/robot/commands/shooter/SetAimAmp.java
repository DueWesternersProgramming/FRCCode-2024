package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class SetAimAmp extends Command {
    
    private Joystick operatorJoystick;
    private final ShooterSubsystem shooter;
    private double power;
    private boolean isFinished = false;
    private PIDController movingPID = new PIDController(0.002,0,0);

     public SetAimAmp(ShooterSubsystem shooter) {
        this.shooter = shooter;
        
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        if (movingPID.atSetpoint()){
            power = movingPID.calculate(shooter.getAimingPosition(),0);
            shooter.setShooterAim(power);
        }
        else{
            isFinished = true;
        }


    }
    
    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
