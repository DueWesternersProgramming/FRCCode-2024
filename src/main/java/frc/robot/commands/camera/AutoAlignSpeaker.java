package frc.robot.commands.camera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;


public class AutoAlignSpeaker extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem visionSubsystem;
    PathPlannerPath path;
    Pose2d targetPose;
    double xPower;
    boolean is_finished = false;
    PIDController xPIDController = new PIDController(VisionConstants.AUTO_ALIGN_P, VisionConstants.AUTO_ALIGN_I, VisionConstants.AUTO_ALIGN_D);

    public AutoAlignSpeaker(DriveSubsystem drive, VisionSubsystem visionSubsystem) {
        this.drive = drive;
        this.visionSubsystem = visionSubsystem;
        addRequirements(drive,visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public void execute() {
        
        xPower = xPIDController.calculate(visionSubsystem.GetTargetHorizontalOffset(), 0);
        if (visionSubsystem.hasValidAprilTag()){
            if ((xPIDController.atSetpoint()) == false) {
                drive.drive(xPower, 0, 0, false, true);
            }
            else{
                is_finished = true;
            }
    }

    }

    @Override
    public void initialize() {
        visionSubsystem.SetActivePipeline(1);

        xPIDController.setTolerance(2);

        // Optional<Alliance> ally = DriverStation.getAlliance();
        
        // if (ally.isPresent()) {
        //     if (ally.get() == Alliance.Red) {
        //         targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
            
        //     }
        
        //     if (ally.get() == Alliance.Blue) {
        //         targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
                
        //     }

        //     PathConstraints constraints = new PathConstraints(1.0, 4.0,Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        //     Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path,constraints,3.0);
        //     CommandScheduler.getInstance().schedule(pathfindingCommand);
        // }
        // else{
        //     DriverStation.reportWarning("Red/Blue Alliance is not avalible",false);
        // }
        
        
    }

    @Override
    public boolean isFinished() {
        return is_finished;
    }
}
