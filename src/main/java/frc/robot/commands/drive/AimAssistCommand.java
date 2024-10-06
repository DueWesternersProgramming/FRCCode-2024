package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CowboyUtils;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.RobotState;

public class AimAssistCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final XboxController driveController;
    PIDController rotController = new PIDController(0.1, 0, 0);
    double targetAngle, output;
    Pose2d target;

    public AimAssistCommand(DriveSubsystem driveSubsystem, XboxController driveController, Pose2d target) {
        rotController.enableContinuousInput(-180, 180);
        this.driveSubsystem = driveSubsystem;
        this.driveController = driveController;
        this.target = target;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        driveSubsystem.drive(0, 0, 0, true, true);
    }

    @Override
    public void execute() {
        double xRaw = -(driveController.getRawAxis(TeleopConstants.DRIVE_COMMAND_X_AXIS));
        double yRaw = -(driveController.getRawAxis(TeleopConstants.DRIVE_COMMAND_Y_AXIS));

        double xConstrained = MathUtil.applyDeadband(
                MathUtil.clamp(xRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_DEADZONE);
        double yConstrained = MathUtil.applyDeadband(
                MathUtil.clamp(yRaw, -TeleopConstants.MAX_SPEED_PERCENT, TeleopConstants.MAX_SPEED_PERCENT),
                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_DEADZONE);

        double xSquared = Math.copySign(xConstrained * xConstrained, xConstrained);
        double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);

        targetAngle = CowboyUtils.getAngleFromPoses(RobotState.robotPose, target);
        output = rotController.calculate(RobotState.robotPose.getRotation().getDegrees(), targetAngle);
        driveSubsystem.drive(xSquared, ySquared, output, true, true);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
