package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotContainer.UserPolicy;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final Joystick joystick;

    public TeleopDriveCommand(DriveSubsystem drive, Joystick joystick) {
        this.drive = drive;
        this.joystick = joystick;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, DrivetrainConstants.FIELD_RELATIVE, true);
    }

    @Override
    public void execute() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED){
            double xRaw = joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_X_AXIS);
            double yRaw = joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_Y_AXIS);
            double rotRaw = -joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_ROT_AXIS);

            double xConstrained = MathUtil.applyDeadband(MathUtil.clamp(xRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);
            double yConstrained = MathUtil.applyDeadband(MathUtil.clamp(yRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);
            double rotConstrained = MathUtil.applyDeadband(
                    MathUtil.clamp(rotRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);

            double xSquared = Math.copySign(xConstrained * xConstrained, xConstrained);
            double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);
            double rotSquared = Math.copySign(rotConstrained * rotConstrained, rotConstrained);

            if (UserPolicy.xLocked) {
                drive.setX();
                return;
            }

            if (UserPolicy.twistable) {
                drive.drive(-ySquared, -xSquared, -rotSquared, DrivetrainConstants.FIELD_RELATIVE, true);
            } 
            
            else {
                if (UserPolicy.canAutoAlign){
                    drive.drive(-ySquared, -xSquared, DriveSubsystem.autoAimSpeed, DrivetrainConstants.FIELD_RELATIVE, true);
                }
                else {
                    drive.drive(-ySquared, -xSquared, 0, DrivetrainConstants.FIELD_RELATIVE, true);
                }
            }
        }
        
    }

    @Override
    public void initialize() {
        drive.drive(0, 0, 0, DrivetrainConstants.FIELD_RELATIVE, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
