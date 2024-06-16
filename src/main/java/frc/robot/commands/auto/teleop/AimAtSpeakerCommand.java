package frc.robot.commands.auto.teleop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AimAtSpeakerCommand extends Command {
    private final DriveSubsystem drive;
    private static final Translation2d targetPosition = new Translation2d(0, 0); // Figure this out later
    private static final ProfiledPIDController rotationPidController = new ProfiledPIDController(1, 0, 0,
            new Constraints(2.0, 3.0));
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

        double angleToTarget = Math.atan2(difference.getY(), difference.getX());

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
        System.out.println(rotationPidController.calculate(currentPose.getRotation().getDegrees(),
                calculateTargetAngle(currentPose)));
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
