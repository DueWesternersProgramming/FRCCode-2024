package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
    private final ShooterSubsystem shooter;
    
     public StartShooter(ShooterSubsystem shooter) {
        this.shooter = shooter ;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.shooterOn();
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
