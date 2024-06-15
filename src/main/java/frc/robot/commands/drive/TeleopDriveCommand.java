package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotContainer.UserPolicy;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.auto.teleop.AimAtSpeakerCommand;
import frc.robot.commands.auto.teleop.AutoAlignSpeaker;

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
        drive.drive(0, 0, 0, DrivetrainConstants.FIELD_RELATIVE, true);
    }

    @Override
    public void execute() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            boolean fieldRelative = DrivetrainConstants.FIELD_RELATIVE;

            double xRaw = -(joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_X_AXIS));
            double yRaw = -(joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_Y_AXIS));

            if (joystick.getRawButton(TeleopConstants.SPEAKER_AIM_BUTTON)) {
                rot = -(AimAtSpeakerCommand.getAimSpeed());
            } else {
                rot = -(joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_ROT_AXIS));
            }

            if (joystick.getRawButton(TeleopConstants.ROBOT_RELATIVE_BUTTON)) {
                fieldRelative = !DrivetrainConstants.FIELD_RELATIVE;
                xRaw = -joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_X_AXIS);
                yRaw = -joystick.getRawAxis(TeleopConstants.DRIVE_COMMAND_Y_AXIS);
            }

            double xConstrained = MathUtil.applyDeadband(
                    MathUtil.clamp(xRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);
            double yConstrained = MathUtil.applyDeadband(
                    MathUtil.clamp(yRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);
            double rotConstrained = MathUtil.applyDeadband(
                    MathUtil.clamp(rot, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                    RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_THRESHOLD);

            double xSquared = Math.copySign(xConstrained * xConstrained, xConstrained);
            double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);
            double rotSquared = Math.copySign(rotConstrained * rotConstrained, rotConstrained);

            if (UserPolicy.xLocked) {
                drive.setX();
                return;
            }

            drive.drive(ySquared, xSquared, rotSquared, fieldRelative, true);

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