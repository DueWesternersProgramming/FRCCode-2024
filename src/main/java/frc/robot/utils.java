package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class utils {

    public static Pose2d blueAllianceSpeaker = new Pose2d(1.4, 5.55, new Rotation2d(Math.toRadians(0)));
    public static Pose2d redAllianceSpeaker = new Pose2d(15.2, 5.55, new Rotation2d(Math.toRadians(180)));

    public static Pose2d getAllainceSpeakerPose() {
        return isRedAlliance() ? redAllianceSpeaker : blueAllianceSpeaker;
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Red) : (false);
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == Alliance.Blue) : (false);
    }
}