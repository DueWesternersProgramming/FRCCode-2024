package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CowboyUtils;
import frc.robot.RobotState;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotConstants.VisionConstants;
import java.util.NoSuchElementException;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends SubsystemBase {

    static AprilTagFieldLayout aprilTagFieldLayout;

    public static Camera[] cameras = new Camera[VisionConstants.CAMERA_NAMES.length];
    public static CameraSim[] cameraSims = new CameraSim[VisionConstants.CAMERA_NAMES.length];
    private VisionSystemSim visionSim;

    public VisionSubsystem() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

            if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {

                if (RobotBase.isSimulation()) {
                    visionSim = new VisionSystemSim("main");
                    visionSim.addAprilTags(CowboyUtils.aprilTagFieldLayout);

                    visionSim.clearCameras();

                    for (int i = 0; i < VisionConstants.CAMERA_NAMES.length; i++) {
                        cameraSims[i] = new CameraSim(VisionConstants.CAMERA_NAMES[i],
                                VisionConstants.CAMERA_POSITIONS[i]);
                        visionSim.addCamera(cameraSims[i].photonCameraSim, VisionConstants.CAMERA_POSITIONS[i]);
                    }

                } else {
                    for (int i = 0; i < VisionConstants.CAMERA_NAMES.length; i++) {
                        cameras[i] = new Camera(VisionConstants.CAMERA_NAMES[i], VisionConstants.CAMERA_POSITIONS[i]);
                    }
                }
            }

        }
    }

    public static Pose2d getVisionPose(int i) throws NoSuchElementException {
        try {
            if (RobotBase.isSimulation()) {
                return cameraSims[i].getEstimatedGlobalPose(RobotState.robotPose)
                        .orElseThrow().estimatedPose
                        .toPose2d();
            } else {
                return cameras[i].getEstimatedGlobalPose(RobotState.robotPose)
                        .orElseThrow().estimatedPose
                        .toPose2d();
            }
        } catch (Exception e) {
            throw e;
        }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            if (RobotBase.isSimulation()) {
                visionSim.update(RobotState.robotPose);
            }
        }
    }
}