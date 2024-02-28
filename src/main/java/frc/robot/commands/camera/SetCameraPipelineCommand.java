package frc.robot.commands.camera;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class SetCameraPipelineCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private int pipe_line;
    public SetCameraPipelineCommand(VisionSubsystem visionSubsystem, int pipeline) {
        pipe_line = pipeline;
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {

    }

    @Override
    public void initialize() {
        visionSubsystem.SetActivePipeline(pipe_line);  
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
