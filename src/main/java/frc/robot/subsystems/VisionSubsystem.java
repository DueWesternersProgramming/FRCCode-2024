package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{

    NetworkTable cameraTableEntry; 
    
    public VisionSubsystem(){

        try{
            cameraTableEntry = NetworkTableInstance.getDefault().getTable("limelight");
            IntializeLimelight();
        }
        catch (Exception e){
            System.out.println("Vision Error: " + e);
        }

    }

    public void IntializeLimelight() {
        SetLedMode(0);
        SetCamMode(0);
        SetActivePipeline(0);
    }

    public boolean HasValidTarget(){
        double tv = cameraTableEntry.getEntry("tv").getDouble(0);
        if(tv == 1){
            return true;
        }

        return false;
    }

    public double GetTargetHorizontalOffset(){
        if(HasValidTarget()){
            return cameraTableEntry.getEntry("tx").getDouble(0);
        }
        return Double.NaN;
    }

    public double GetTargetVerticalOffset(){
        if (HasValidTarget()) {
            return cameraTableEntry.getEntry("ty").getDouble(0);
        }
        return Double.NaN;
    }

    public double GetTargetArea(){
        if(HasValidTarget()){
            return cameraTableEntry.getEntry("ta").getDouble(0);
        }
        return Double.NaN;
    }

    public double GetCamMode(){
        return cameraTableEntry.getEntry("camMode").getDouble(0);
    }

    public double GetLedMode(){
        return cameraTableEntry.getEntry("ledMode").getDouble(0);
    }

    public double getActivePipeline(){
        return cameraTableEntry.getEntry("getpipe").getDouble(0);
    }

    public void SetCamMode(double camMode){
        cameraTableEntry.getEntry("camMode").setNumber(camMode);
    }

    public void SetLedMode(double ledMode){
        cameraTableEntry.getEntry("ledMode").setNumber(ledMode);
    }

    public void SetActivePipeline(int pipeline){
        cameraTableEntry.getEntry("pipeline").setNumber(pipeline);
    }

    public boolean hasValidAprilTag(){
        double result = cameraTableEntry.getEntry("getpipe").getDouble(-1);
        
        if (result != -1){
            return true;
        }
        else{
            return false;
        }
    }
    public double getTargetID(){
        try{
            return cameraTableEntry.getEntry("getpipe").getDouble(-1);
        }
        catch(Exception e){
            return -1;
        }
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("tx", cameraTableEntry.getEntry("tx").getDouble(0));
        SmartDashboard.putNumber("pipeline", cameraTableEntry.getEntry("getpipe").getDouble(-1));
    }
}