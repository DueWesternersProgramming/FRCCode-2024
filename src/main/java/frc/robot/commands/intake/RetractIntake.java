package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends Command {
    private final IntakeSubsystem intake;
    private boolean finished;
    
     public RetractIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        finished = false;  
    }
    
    @Override
    public void execute() {
        if (intake.getIntakeDeploymentEncoderPosition() >= IntakeConstants.INTAKE_RETRACTED_POSITION){
            intake.setIntakeDeploymentMotorSpeed(-IntakeConstants.INTAKE_DEPLOYMENT_SPEED);
          }
          else{
            finished = true;
          }
    }
    
    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
