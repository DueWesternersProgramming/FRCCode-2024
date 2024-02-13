package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
    private final IntakeSubsystem intake;
    private boolean finished;
    
     public DeployIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        finished = false;  
    }
    
    @Override
    public void execute() {
        if (intake.getIntakeDeploymentEncoderPosition() < IntakeConstants.INTAKE_DEPLOYED_POSITION){
            intake.setIntakeDeploymentMotorSpeed(IntakeConstants.INTAKE_DEPLOYMENT_SPEED);
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
