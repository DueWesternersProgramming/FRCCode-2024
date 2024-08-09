package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    static PhotonCamera backLeftCamera;
    private static PhotonCamera backRightCamera;

    static AprilTagFieldLayout aprilTagFieldLayout;

    private static PhotonPoseEstimator backLeftPoseEstimator;
    private static PhotonPoseEstimator backRightPoseEstimator;

    public VisionSubsystem() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

            // Create as many camera instances (photonvision) as cameras you have

            backLeftCamera = new PhotonCamera("backLeftCamera");
            backRightCamera = new PhotonCamera("backRightCamera");

            backLeftPoseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.LOWEST_AMBIGUITY,
                    backLeftCamera,
                    new Transform3d(new Translation3d(
                            Units.inchesToMeters(-13), // forward+
                            Units.inchesToMeters(12.75), // left+
                            Units.inchesToMeters(8.5)), // up+
                            new Rotation3d(
                                    Units.degreesToRadians(0),
                                    Units.degreesToRadians(-30), // Note, these are all counter clockwise so to face up
                                                                 // we need -40 ;)
                                    Units.degreesToRadians(180 - 15))));

            backRightPoseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.LOWEST_AMBIGUITY,
                    backRightCamera,
                    new Transform3d(new Translation3d(
                            Units.inchesToMeters(-13), // forward+
                            Units.inchesToMeters(-12.75), // left+
                            Units.inchesToMeters(8.5)), // up+

                            new Rotation3d(
                                    Units.degreesToRadians(0),
                                    Units.degreesToRadians(-30), // Note, these are all counter clockwise so to face up
                                                                 // we need -40 ;)
                                    Units.degreesToRadians(180 + 15))));

        }
    }

    public static PhotonCamera getBackLeftPhotonCamera() {
        return backLeftCamera;
    }

    public static PhotonCamera getBackRightPhotonCamera() {
        return backRightCamera;
    }

    public static PhotonPoseEstimator getBackLeftPhotonPoseEstimator() {
        return backLeftPoseEstimator;
    }

    public static PhotonPoseEstimator getBackRightPhotonPoseEstimator() {
        return backRightPoseEstimator;
    }

    public void setPipeline(PhotonCamera camera, int index) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            camera.setPipelineIndex(index);
        }
    }

    public PhotonPipelineResult getResult(PhotonCamera camera) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return camera.getLatestResult();
        } else {
            return null;
        }
    }

    public boolean isCameraConnected(PhotonCamera camera) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return camera.isConnected();
        } else {
            return false;
        }
    }

    public static boolean hasResults(PhotonCamera camera) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return camera.getLatestResult().hasTargets();
        } else {
            return false;
        }
    }

    public PhotonTrackedTarget getBestTarget(PhotonCamera camera) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return getResult(camera).getBestTarget();
        } else {
            return null;
        }
    }

    public double getTargetYaw(PhotonCamera camera) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            return getBestTarget(camera).getYaw();
        } else {
            return 0;
        }
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator estimator,
            PhotonCamera camera, Pose2d prevEstimatedRobotPose) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            estimator.setReferencePose(prevEstimatedRobotPose);
            return estimator.update();
        } else {
            return null;
        }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            SmartDashboard.putBoolean("Left Has a tracked object:", hasResults(backLeftCamera));
            SmartDashboard.putBoolean("Right Has a tracked object:", hasResults(backRightCamera));

        }
    }
}