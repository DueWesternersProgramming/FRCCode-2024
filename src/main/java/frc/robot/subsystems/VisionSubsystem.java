package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase{
    PhotonCamera camera = new PhotonCamera("photonvision");

    public VisionSubsystem(){

    }

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