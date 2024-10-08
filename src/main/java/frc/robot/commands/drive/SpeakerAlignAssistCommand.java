package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CowboyUtils;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriverAssistConstants;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.RobotState;

public class SpeakerAlignAssistCommand extends Command {
        private final DriveSubsystem driveSubsystem;
        private final XboxController driveController;
        PIDController xController = new PIDController(2, 0, 0);
        PIDController yController = new PIDController(2, 0, 0);
        PIDController rotController = new PIDController(.1, 0, 0);
        double targetAngle, output;
        Pose2d target;

        public SpeakerAlignAssistCommand(DriveSubsystem driveSubsystem, XboxController driveController) {
                rotController.enableContinuousInput(-180, 180);
                rotController.setTolerance(10);
                this.driveSubsystem = driveSubsystem;
                this.driveController = driveController;
                addRequirements(driveSubsystem);
        }

        @Override
        public void initialize() {
                target = CowboyUtils.getAllianceSpeaker();
                driveSubsystem.drive(0, 0, 0, true, true);
        }

        @Override
        public void execute() {
                double multiplier = driveController.getRawAxis(TeleopConstants.SPEAKER_ASSIST_AXIS);
                double maxSpeedPercent = (DriverAssistConstants.MAX_VELOCITY
                                / DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND) / 2;
                double xOutput = MathUtil.clamp(xController.calculate(RobotState.robotPose.getY(), target.getY()),
                                -maxSpeedPercent,
                                maxSpeedPercent)
                                * multiplier;
                double yOutput = MathUtil.clamp(yController.calculate(RobotState.robotPose.getX(), target.getX()),
                                -maxSpeedPercent,
                                maxSpeedPercent)
                                * multiplier;
                double rotOutput = rotController.calculate(RobotState.robotPose.getRotation().getDegrees(),
                                target.getRotation().getDegrees());

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
