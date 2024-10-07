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

public class AlignAssistCommand extends Command {
        private final DriveSubsystem driveSubsystem;
        private final XboxController driveController;
        PIDController xController = new PIDController(2, 0, 0);
        PIDController yController = new PIDController(2, 0, 0);
        PIDController rotController = new PIDController(.1, 0, 0);
        double targetAngle, output;
        Pose2d target;

        public AlignAssistCommand(DriveSubsystem driveSubsystem, XboxController driveController, Pose2d target) {
                rotController.enableContinuousInput(-180, 180);
                rotController.setTolerance(10);
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
                double multiplier = driveController.getRawAxis(TeleopConstants.SPEAKER_ASSIST_AXIS);
                double xInputRaw = (driveController.getRawAxis(TeleopConstants.DRIVE_COMMAND_X_AXIS)); // We keep these
                                                                                                       // positive
                                                                                                       // here because
                                                                                                       // we end up
                                                                                                       // not needing to
                double yInputRaw = (driveController.getRawAxis(TeleopConstants.DRIVE_COMMAND_Y_AXIS)); // make them
                                                                                                       // negitive
                                                                                                       // down later in
                                                                                                       // the
                                                                                                       // assist logic.
                double xConstrained = MathUtil.applyDeadband(
                                MathUtil.clamp(xInputRaw, -TeleopConstants.MAX_SPEED_PERCENT,
                                                TeleopConstants.MAX_SPEED_PERCENT),
                                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_DEADZONE);
                double yConstrained = MathUtil.applyDeadband(
                                MathUtil.clamp(yInputRaw, -TeleopConstants.MAX_SPEED_PERCENT,
                                                TeleopConstants.MAX_SPEED_PERCENT),
                                RobotConstants.PortConstants.Controller.JOYSTICK_AXIS_DEADZONE);

                double xOutput = MathUtil.clamp(xController.calculate(RobotState.robotPose.getY(), target.getY()), -1,
                                1)
                                * multiplier;
                double yOutput = MathUtil.clamp(yController.calculate(RobotState.robotPose.getX(), target.getX()), -1,
                                1)
                                * multiplier;
                double rotOutput = rotController.calculate(RobotState.robotPose.getRotation().getDegrees(),
                                target.getRotation().getDegrees());

                // xOutput = (xConstrained != 0) ? xConstrained : xOutput;
                // yOutput = (yConstrained != 0) ? yConstrained : yOutput; // Uncomment for
                // manual adjustment

                targetAngle = CowboyUtils.getAngleFromPoses(RobotState.robotPose, target);

                driveSubsystem.drive(CowboyUtils.isBlueAlliance() ? xOutput : -xOutput,
                                CowboyUtils.isBlueAlliance() ? yOutput : -yOutput, rotOutput, true, true);
        }

        @Override
        public void end(boolean interrupted) {

        }

        @Override
        public boolean isFinished() {
                return false;
        }

}
