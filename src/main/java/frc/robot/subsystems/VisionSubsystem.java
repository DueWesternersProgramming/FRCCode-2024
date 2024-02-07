package frc.robot.subsystems;

import org.opencv.objdetect.FaceDetectorYN;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
 
    NetworkTable limelightTableEntry; 
    
    public VisionSubsystem(){

        try{
            limelightTableEntry = NetworkTableInstance.getDefault().getTable("limelight");
            IntializeLimelight();
        }
        catch (Exception e){
            System.out.println("Vision Error: " + e);
        }

    }

    public void IntializeLimelight()
    {
        SetLedMode(0);
        SetCamMode(0);
        SetActivePipeline(0);
    }

    public boolean HasValidTarget()
    {
        double tv = limelightTableEntry.getEntry("tv").getDouble(0);

        if(tv == 1)
        {
            return true;
        }

        return false;
    }

    public double GetTargetHorizontalOffset()
    {
        if(HasValidTarget())
        {
            return limelightTableEntry.getEntry("tx").getDouble(0);
        }

        return Double.NaN;
    }

    public double GetTargetVerticalOffset()
    {
        if(HasValidTarget())
        {
            return limelightTableEntry.getEntry("ty").getDouble(0);
        }

        return Double.NaN;
    }

    public double GetTargetArea()
    {
        if(HasValidTarget())
        {
            return limelightTableEntry.getEntry("ta").getDouble(0);
        }

        return Double.NaN;
    }

    public double GetCamMode()
    {
        return limelightTableEntry.getEntry("camMode").getDouble(0);
    }

    public double GetLedMode()
    {
        return limelightTableEntry.getEntry("ledMode").getDouble(0);
    }

    public double getActivePipeline()
    {
        return limelightTableEntry.getEntry("getpipe").getDouble(0);
    }

    public void SetCamMode(double camMode)
    {
        limelightTableEntry.getEntry("camMode").setNumber(camMode);
    }

    public void SetLedMode(double ledMode)
    {
        limelightTableEntry.getEntry("ledMode").setNumber(ledMode);
    }

    public void SetActivePipeline(int pipeline)
    {
        limelightTableEntry.getEntry("pipeline").setNumber(pipeline);
    }
    public boolean hasValidAprilTag(){
        double result = limelightTableEntry.getEntry("getpipe").getDouble(-1);
        
        if (result != -1){
            return true;
        }
        else{
            return false;
        }
    }
    public double getTargetID(){
        try{
            return limelightTableEntry.getEntry("getpipe").getDouble(-1);
        }
        catch(Exception e){
            return -1;
        }
    }
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("tv", limelightTableEntry.getEntry("tv").getDouble(0));
        SmartDashboard.putNumber("tx", limelightTableEntry.getEntry("tx").getDouble(0));
        // SmartDashboard.putNumber("ty", limelightTableEntry.getEntry("ty").getDouble(0));
        // SmartDashboard.putNumber("ta", limelightTableEntry.getEntry("ta").getDouble(0));
        // SmartDashboard.putNumber("camMode", limelightTableEntry.getEntry("camMode").getDouble(-1));
        // SmartDashboard.putNumber("ledmode", limelightTableEntry.getEntry("ledMode").getDouble(-1));
        SmartDashboard.putNumber("pipeline", limelightTableEntry.getEntry("getpipe").getDouble(-1));
    }
}