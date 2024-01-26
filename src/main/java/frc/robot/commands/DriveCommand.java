package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotContainer.UserPolicy;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    private final Joystick joystick;

    public DriveCommand(DriveSubsystem drive, Joystick joystick) {
        this.drive = drive;
        this.joystick = joystick;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, DrivetrainConstants.kFieldRelative, true);
    }

    @Override
    public void execute() {
        double xRaw = joystick.getRawAxis(0);
        double yRaw = joystick.getRawAxis(1);
        double rotRaw = -joystick.getRawAxis(2);

        double xConstrained = MathUtil.applyDeadband(MathUtil.clamp(xRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                RobotConstants.PortConstants.CONTROLLER.JOYSTICK_AXIS_THRESHOLD);
        double yConstrained = MathUtil.applyDeadband(MathUtil.clamp(yRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                RobotConstants.PortConstants.CONTROLLER.JOYSTICK_AXIS_THRESHOLD);
        double rotConstrained = MathUtil.applyDeadband(
                MathUtil.clamp(rotRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                RobotConstants.PortConstants.CONTROLLER.JOYSTICK_AXIS_THRESHOLD);

        double xSquared = Math.copySign(xConstrained * xConstrained, xConstrained);
        double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);
        double rotSquared = Math.copySign(rotConstrained * rotConstrained, rotConstrained);

        if (UserPolicy.xLocked) {
            drive.setX();
            return;
        }

        if (UserPolicy.twistable) {
            drive.drive(-ySquared, -xSquared, -rotSquared, DrivetrainConstants.kFieldRelative, true);
        } else {
            drive.drive(-ySquared, -xSquared, 0, DrivetrainConstants.kFieldRelative, true);
        }
    }

    @Override
    public void initialize() {
        drive.drive(0, 0, 0, DrivetrainConstants.kFieldRelative, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
