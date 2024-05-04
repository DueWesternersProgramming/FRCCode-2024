package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends Command {
    private final ShooterSubsystem shooter;
    
     public StopShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.shooterOff();
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
