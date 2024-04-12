package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

    static PhotonCamera camera;
    static AprilTagFieldLayout aprilTagFieldLayout;
    static Transform3d robotToCam;
    static PhotonPoseEstimator photonPoseEstimator;
    
    public VisionSubsystem(){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            camera = new PhotonCamera("photonvision");
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
        }
    }

    public void setPipeline(int index){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            camera.setPipelineIndex(index);
        }
    }

    public PhotonPipelineResult getResult(){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return camera.getLatestResult();
        }
        else {
            return null;
        }
    }

    public boolean isCameraConnected(){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return camera.isConnected();
        }
        else {
            return false;
        }
    }
    
    public boolean hasResults() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return camera.getLatestResult().hasTargets();
        }
        else {
            return false;
        }
    }

    public PhotonTrackedTarget getBestTarget(){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return getResult().getBestTarget();
        }
        else {
            return null;
        }
    }

    public double getTargetYaw(){
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            return getBestTarget().getYaw();
        }
        else {
            return 0;
        }
    }
    
    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimator.update();
        }
        else {
            return null;
        }
    }
    
    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED){
            SmartDashboard.putBoolean("Has a tracked object:", hasResults());
        }
    }
}