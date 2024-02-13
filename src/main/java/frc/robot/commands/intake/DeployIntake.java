package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
    private final IntakeSubsystem intake;
    
     public DeployIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeDeploymentMotorSpeed(0.05);
        while (intake.getIntakeDeploymentEncoderPosition() <= 100) {}
        
        if (intake.getIntakeDeploymentEncoderPosition() < IntakeConstants.INTAKE_DEPLOYED_POSITION){
            intake.setIntakeDeploymentMotorSpeed(IntakeConstants.INTAKE_DEPLOYMENT_SPEED);
          }
          else{
            finished = true;
          }
    }
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
