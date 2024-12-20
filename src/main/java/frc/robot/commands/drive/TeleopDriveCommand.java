package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.RobotState;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TeleopDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final XboxController joystick;
    private double rot = 0;

    public TeleopDriveCommand(DriveSubsystem drive, XboxController joystick) {
        this.drive = drive;
        this.joystick = joystick;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0, 0.0, DrivetrainConstants.FIELD_RELATIVE, true);
    }

    @Override
    public void execute() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            boolean fieldRelative = DrivetrainConstants.FIELD_RELATIVE;

            double xRaw = -(joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_X_AXIS));
            double yRaw = -(joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_Y_AXIS));

            rot = -(joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_ROT_AXIS));

            if (joystick.getRawButton(TeleopConstants.ROBOT_RELATIVE_BUTTON)) {
                fieldRelative = !DrivetrainConstants.FIELD_RELATIVE;
                xRaw = -joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_X_AXIS);
                yRaw = -joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_Y_AXIS);
            }

            double xConstrained = MathUtil.applyDeadband(
                    MathUtil.clamp(xRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_DEADZONE);
            double yConstrained = MathUtil.applyDeadband(
                    MathUtil.clamp(yRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_DEADZONE);
            double rotConstrained = MathUtil.applyDeadband(
                    MathUtil.clamp(rot, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_DEADZONE);

            double xSquared = Math.copySign(xConstrained * xConstrained, xConstrained);
            double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);
            double rotSquared = Math.copySign(rotConstrained * rotConstrained, rotConstrained);

            if (RobotState.xLocked) {
                drive.setX();
                return;
            }

            drive.drive(xSquared, ySquared, rotSquared, fieldRelative, true);

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