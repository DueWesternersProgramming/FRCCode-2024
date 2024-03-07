package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ServoCommand extends Command {
    private final ShooterSubsystem shooter;
    private final int mode;
    
     public ServoCommand(ShooterSubsystem shooter, int mode) {
        this.shooter = shooter;
        this.mode = mode;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (mode == 0) { // speaker
            shooter.setServoPosition(ShooterConstants.SERVO_L_BEGINNING_POSITION, ShooterConstants.SERVO_R_BEGINNING_POSITION);
        }
        else if (mode == 1) { // amp
            shooter.setServoPosition(ShooterConstants.SERVO_L_SHOOTING_POSITION, ShooterConstants.SERVO_R_SHOOTING_POSITION);
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
