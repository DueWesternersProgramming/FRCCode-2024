// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.Waypoint;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.light.LEDHasNoteUpdater;
import frc.robot.commands.light.LEDOff;
import frc.robot.commands.light.LEDPrematch;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CowboyUtils;
import frc.robot.RobotConstants.DriverAssistConstants;
import frc.robot.RobotConstants.FieldPointPoses;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.ShootingCommands;
import frc.robot.commands.drive.LaneAssistCommand;
import frc.robot.commands.drive.SourceAimAssistCommand;
import frc.robot.commands.drive.SpeakerAlignAssistCommand;
import frc.robot.commands.NoteMovementCommands;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final TransitSubsystem transitSubsystem = new TransitSubsystem();
    public final LightSubsystem lightSubsystem = new LightSubsystem();
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();

    private final XboxController driveJoystick = new XboxController(
            RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
    private final Joystick operatorJoystick = new Joystick(RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);

    SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();
    static SendableChooser<List<Waypoint>> m_laneChooser = new SendableChooser<List<Waypoint>>();

    PowerDistribution pdp;

    private final Field2d field = new Field2d();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));
        climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem, operatorJoystick));
        lightSubsystem.setDefaultCommand(new LEDHasNoteUpdater(lightSubsystem, intakeSubsystem));

        createNamedCommands();

        configureTriggers();

        try {
            pdp = new PowerDistribution(16, ModuleType.kRev);
            DogLog.setPdh(pdp);
            m_autoPositionChooser = AutoBuilder.buildAutoChooser("JustShoot");
            addLaneSelections();
            Shuffleboard.getTab("Auto and setup").add("Auto Chooser", m_autoPositionChooser);
            Shuffleboard.getTab("Auto and setup").add("Teleop Lane Chooser", m_laneChooser);
            Shuffleboard.getTab("Power").add(pdp);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void addLaneSelections() {
        if (CowboyUtils.isBlueAlliance()) {
            m_laneChooser.addOption("Left Lane", FieldPointPoses.BlueAlliance.LEFT_LANE_WAYPOINTS);
            m_laneChooser.addOption("Middle Lane", FieldPointPoses.BlueAlliance.MIDDLE_LANE_WAYPOINTS);
        } else {
            m_laneChooser.addOption("Left Lane", FieldPointPoses.BlueAlliance.LEFT_LANE_WAYPOINTS);
            m_laneChooser.addOption("Middle Lane", FieldPointPoses.BlueAlliance.MIDDLE_LANE_WAYPOINTS);

            m_laneChooser.setDefaultOption("Middle Lane", FieldPointPoses.BlueAlliance.MIDDLE_LANE_WAYPOINTS);
        }
    }

    public static List<Waypoint> getSelectedLane() {

        return m_laneChooser.getSelected() != null ? m_laneChooser.getSelected()
                : FieldPointPoses.BlueAlliance.MIDDLE_LANE_WAYPOINTS;
    }

    private void createNamedCommands() {

        NamedCommands.registerCommand("StopIntaking",
                NoteMovementCommands.stopIntakingCommand(shooterSubsystem, intakeSubsystem, transitSubsystem));

        NamedCommands.registerCommand("StartIntaking",
                NoteMovementCommands.startIntakingCommand(shooterSubsystem, intakeSubsystem, transitSubsystem,
                        lightSubsystem));

        NamedCommands.registerCommand("ShootSpeaker",
                ShootingCommands.fullShootSpeakerCommand(shooterSubsystem, transitSubsystem, intakeSubsystem,
                        lightSubsystem));
    }

    private void configureTriggers() {

        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            new JoystickButton(driveJoystick, TeleopConstants.RESET_GYRO_BUTTON).onTrue(driveSubsystem.gyroReset());
            new JoystickButton(driveJoystick, TeleopConstants.X_LOCK_BUTTON).onTrue((driveSubsystem.gyroReset()));

            new Trigger(() -> driveJoystick.getRawAxis(2) > 0.25)
                    .whileTrue(new SourceAimAssistCommand(driveSubsystem, driveJoystick));

            new Trigger(() -> driveJoystick.getRawAxis(3) > 0.25)
                    .whileTrue(new SequentialCommandGroup(new LaneAssistCommand().onlyIf(() -> (CowboyUtils
                            .getPoseDistance(
                                    CowboyUtils.getAllianceSpeaker()) > DriverAssistConstants.SKIP_LANE_PATH_DISTANCE)),
                            new SpeakerAlignAssistCommand(driveSubsystem, driveJoystick)));

        }
        // Above = DriveJoystick, Below = OperatorJoystick
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            new JoystickButton(operatorJoystick, 2).onTrue(
                    (NoteMovementCommands.startIntakingCommand(shooterSubsystem, intakeSubsystem, transitSubsystem,
                            lightSubsystem)))
                    .onFalse(NoteMovementCommands.stopIntakingCommand(shooterSubsystem, intakeSubsystem,
                            transitSubsystem));

            new JoystickButton(operatorJoystick, 7)
                    .onTrue(NoteMovementCommands.reverseIntakingCommand(shooterSubsystem, intakeSubsystem,
                            transitSubsystem))
                    .onFalse(NoteMovementCommands.stopIntakingCommand(shooterSubsystem, intakeSubsystem,
                            transitSubsystem));
        }
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            new JoystickButton(operatorJoystick, 4)
                    .onTrue(ShootingCommands.ShootSpeakerChamber(shooterSubsystem, transitSubsystem, intakeSubsystem,
                            lightSubsystem).onlyIf(() -> !RobotState.shootCommandLocked))
                    .onFalse(ShootingCommands.ShootSpeaker(shooterSubsystem, transitSubsystem, intakeSubsystem,
                            lightSubsystem).onlyIf(() -> RobotState.shootCommandLocked));
        }
    }

    public Command preLEDCommand() {
        return new LEDPrematch(lightSubsystem);
    }

    public Command startLEDCommand() {
        return new LEDHasNoteUpdater(lightSubsystem, intakeSubsystem);
    }

    public Command stopLEDCommand() {
        return new LEDOff(lightSubsystem);
    }

    public Command getAutonomousCommand() {
        if (m_autoPositionChooser.getSelected() != null) {
            return m_autoPositionChooser.getSelected();
        } else {
            return driveSubsystem.gyroReset();
        }
    }

    public Command getTestingCommand() {
        return new RobotSystemsCheckCommand(driveSubsystem, shooterSubsystem, intakeSubsystem, transitSubsystem);
    }

    public Field2d getField() {
        return field;
    }
}
