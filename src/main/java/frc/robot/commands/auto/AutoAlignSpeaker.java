package frc.robot.commands.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;


public class AutoAlignSpeaker extends Command {
    private PathPlannerPath bluePath;
    private PathPlannerPath redPath;

    public AutoAlignSpeaker() {
        addRequirements();
    }

    @Override
    public void end(boolean interrupted) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            AutoBuilder.followPath(bluePath).cancel();
        else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            AutoBuilder.followPath(redPath).cancel();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            List<Translation2d> redShot = PathPlannerPath.bezierFromPoses(
                new Pose2d(13.97,5.3, Rotation2d.fromDegrees(-180)),
                new Pose2d(15.36, 4.81, Rotation2d.fromDegrees(-143.62)));

            redPath = new PathPlannerPath(
                redShot,
                new PathConstraints(1, 3.0, 2 * Math.PI, 4 * Math.PI),
                new GoalEndState(0.0, Rotation2d.fromDegrees(-143.62)));
            AutoBuilder.followPath(redPath).schedule();
        }

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            List<Translation2d> blueShot = PathPlannerPath.bezierFromPoses(
                new Pose2d(1.2, 4.79, Rotation2d.fromDegrees(-38.66)));

            bluePath = new PathPlannerPath(
                blueShot,
                new PathConstraints(1, 3.0, 2 * Math.PI, 4 * Math.PI),
                new GoalEndState(0.0, Rotation2d.fromDegrees(-38.66)));
                bluePath.preventFlipping = true;
            AutoBuilder.followPath(bluePath).schedule();
            
            
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
