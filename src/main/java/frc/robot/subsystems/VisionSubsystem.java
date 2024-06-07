package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

public class VisionSubsystem extends SubsystemBase{

    static PhotonCamera frontLeftCamera;
    private static PhotonCamera frontRightCamera;

    static AprilTagFieldLayout aprilTagFieldLayout;

    private static PhotonPoseEstimator frontLeftPoseEstimator;
    private static PhotonPoseEstimator frontRightPoseEstimator;
    
    public VisionSubsystem(){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

            //Create as many camera instances (photonvision) as cameras you have

            frontLeftCamera = new PhotonCamera("frontLeftCamera");
            frontRightCamera = new PhotonCamera("frontRightCamera");

            frontLeftPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.LOWEST_AMBIGUITY,
                frontLeftCamera,
                new Transform3d(new Translation3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(0),  //Figure this out later.
                    Units.inchesToMeters(0)),
                new Rotation3d()));

            frontRightPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.LOWEST_AMBIGUITY,
                frontRightCamera,
                new Transform3d(new Translation3d(
                    Units.inchesToMeters(0),  //Figure this out later.
                    Units.inchesToMeters(0),  //positioning relitive to center of the robot, on the floor. Use CAD mesurements ;)
                    Units.inchesToMeters(0)),
                new Rotation3d()));

        }
    }

    public static PhotonCamera getFrontLeftPhotonCamera(){
        return frontLeftCamera;
    }
    public static PhotonCamera getFrontRightPhotonCamera(){
        return frontRightCamera;
    }

    public static PhotonPoseEstimator getFrontLeftPhotonPoseEstimator(){
        return frontLeftPoseEstimator;
    }
    public static PhotonPoseEstimator getFrontRightPhotonPoseEstimator(){
        return frontRightPoseEstimator;
    }

    public void setPipeline(PhotonCamera camera, int index){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            camera.setPipelineIndex(index);
        }
    }

    public PhotonPipelineResult getResult(PhotonCamera camera){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return camera.getLatestResult();
        }
        else {
            return null;
        }
    }

    public boolean isCameraConnected(PhotonCamera camera){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return camera.isConnected();
        }
        else {
            return false;
        }
    }
    
    public static boolean hasResults(PhotonCamera camera) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return camera.getLatestResult().hasTargets();
        }
        else {
            return false;
        }
    }

    public PhotonTrackedTarget getBestTarget(PhotonCamera camera){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return getResult(camera).getBestTarget();
        }
        else {
            return null;
        }
    }

    public double getTargetYaw(PhotonCamera camera){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return getBestTarget(camera).getYaw();
        }
        else {
            return 0;
        }
    }
    
    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator estimator, PhotonCamera camera, Pose2d prevEstimatedRobotPose) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            estimator.setReferencePose(prevEstimatedRobotPose);
            return estimator.update();
        }
        else {
            return null;
        }
    }
    
    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            SmartDashboard.putBoolean("Left Has a tracked object:", hasResults(frontLeftCamera));
            SmartDashboard.putBoolean("Right Has a tracked object:", hasResults(frontRightCamera));
        
        }
    }
}