package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
    private final ShooterSubsystem shooter;
    private final int mode;
    
     public StartShooter(ShooterSubsystem shooter, int mode) {
        this.shooter = shooter;
        this.mode = mode;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.shooterOn(mode);
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
