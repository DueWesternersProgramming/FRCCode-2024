package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants.FieldPointPoses;

public class CowboyUtils {
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Red) : (false);
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Blue) : (false);
    }

    public static Pose2d getAllianceSource() {
        return isBlueAlliance() ? FieldPointPoses.BLUE_ALLIANCE_SOURCE : FieldPointPoses.RED_ALLIANCE_SOURCE;
    }

    public static Pose2d getAllianceSpeaker() {
        return isBlueAlliance() ? FieldPointPoses.BLUE_ALLIANCE_SPEAKER : FieldPointPoses.RED_ALLIANCE_SPEAKER;
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