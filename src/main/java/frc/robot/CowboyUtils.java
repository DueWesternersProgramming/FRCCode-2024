package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CowboyUtils {
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static Pose2d blueAllianceSpeaker = new Pose2d(1.2, 5.55, new Rotation2d(Math.toRadians(0))); // Meters
    public static Pose2d redAllianceSpeaker = new Pose2d(15.4, 5.55, new Rotation2d(Math.toRadians(180))); // Meters

    public static Pose2d blueAllianceSource = new Pose2d(15.7, 0.55, new Rotation2d()); // Meters
    public static Pose2d redAllianceSource = new Pose2d(0.85, 0.55, new Rotation2d()); // Meters

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Red) : (false);
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Blue) : (false);
    }

    public static Pose2d getAllianceSource() {
        return isBlueAlliance() ? blueAllianceSource : redAllianceSource;
    }

    public static Pose2d getAllianceSpeaker() {
        return isBlueAlliance() ? blueAllianceSpeaker : redAllianceSpeaker;
    }

    public static double getAngleFromPoses(Pose2d robot, Pose2d target) {
        double angle = Math.toDegrees(Math.atan2(target.getY() - robot.getY(), target.getX() - robot.getX()));
        return angle;
    }

    public double getPoseDistance(Pose2d target) {
        double distance = RobotState.robotPose.getTranslation().getDistance(target.getTranslation());
        return distance;
    }
}