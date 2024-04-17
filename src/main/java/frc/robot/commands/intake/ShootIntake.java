package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootIntake extends Command {
    private final IntakeSubsystem intake;
    
     public ShootIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeShoot();
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
