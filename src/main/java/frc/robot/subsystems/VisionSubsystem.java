package frc.robot.subsystems;
import frc.robot.Robot;
import frc.robot.RobotConstants.VisionConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase{
    private PhotonCamera camera;
    public VisionSubsystem(){
        try {
            camera = new PhotonCamera(VisionConstants.PHOTON_CAMERA_NAME);
        }
        catch (NullPointerException e) {
            System.out.println("Vision subsytem error:"+e);
        }
    }

    public int getAprilTagID() {
        return getBestTarget().getFiducialId();
    }

    // private static boolean isValidTagId(int tagId) {
    //     return tagId > 0 && tagId < 9;
    // }

    public void setPipeline(int index){
        camera.setPipelineIndex(index);
    }

    public PhotonPipelineResult getResult(){
        return camera.getLatestResult();
    }

    public boolean isCameraConnected(){
        return camera.isConnected();
        
    }
    
    public boolean hasResults() {
        return getResult().hasTargets();
    }
    public PhotonTrackedTarget getBestTarget(){

        return getResult().getBestTarget();
        
    }
    public double getTargetYaw(){
        return getBestTarget().getYaw();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has a tracked object:", hasResults());
    }
}