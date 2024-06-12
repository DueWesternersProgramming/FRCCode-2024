package frc.robot.commands.auto.teleop;

import org.opencv.core.Point;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.System;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotContainer.UserPolicy;

public class AimAtSpeakerCommand extends Command {
    private final DriveSubsystem drive;
    private final static Translation2d targetPosition = new Translation2d(0, 0);// figure this out later.
    private static PIDController rotationPidController = new PIDController(1, 0, 0);

    private static Pose2d currentPose;

    public AimAtSpeakerCommand(DriveSubsystem drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    public static double getAimSpeed() {
        currentPose = DriveSubsystem.getPose().get();
        return rotationPidController.calculate(currentPose.getRotation().getDegrees(),
                calculateTargetAngle(currentPose));
    }

    private static double calculateTargetAngle(Pose2d currentPose) {
        Translation2d difference = targetPosition.minus(currentPose.getTranslation());

        // Calculate the angle to the target from the field's reference frame
        double angleToTarget = Math.atan2(difference.getY(), difference.getX());

        // Convert the field reference angle to the robot's reference frame
        double robotHeading = currentPose.getRotation().getRadians();
        double targetAngle = Math.toDegrees(angleToTarget - robotHeading);

        // Normalize the angle to the range [-180, 180]
        targetAngle = Math.IEEEremainder(targetAngle, 360.0);

        return targetAngle;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        // System.out.println(rotationPidController.calculate(currentPose.getRotation().getDegrees(),calculateTargetAngle(currentPose)));
    }

    @Override
    public void initialize() {

        rotationPidController.disableContinuousInput();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}