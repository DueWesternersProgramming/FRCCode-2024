// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.ShootingCommands;
import frc.robot.commands.drive.AlignWithPose;
import frc.robot.commands.IntakingCommands;

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

    PowerDistribution pdp;

    private final Field2d field = new Field2d();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));
        climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem, operatorJoystick));
        lightSubsystem.setDefaultCommand(new LEDHasNoteUpdater(lightSubsystem, intakeSubsystem));

        createNamedCommands();

        configureButtonBindings();

        try {
            pdp = new PowerDistribution(16, ModuleType.kRev);
            DogLog.setPdh(pdp);
            m_autoPositionChooser = AutoBuilder.buildAutoChooser("");
            Shuffleboard.getTab("Autonomous").add(m_autoPositionChooser);
            Shuffleboard.getTab("Power").add(pdp);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void createNamedCommands() {
        NamedCommands.registerCommand("StartShooter",
                shooterSubsystem.startShooterCommand());

        NamedCommands.registerCommand("StopShooter",
                shooterSubsystem.stopShooterCommand());

        // NamedCommands.registerCommand("StartIntake",
        // intakeSubsystem.startIntakeCommand());

        // NamedCommands.registerCommand("StopIntake",
        // intakeSubsystem.stopIntakeCommand());

        NamedCommands.registerCommand("StartTransit", transitSubsystem.startTransitCommand());

        NamedCommands.registerCommand("StopTransit", transitSubsystem.startTransitCommand());

        NamedCommands.registerCommand("StopIntaking",
                IntakingCommands.stopIntakingCommand(shooterSubsystem, intakeSubsystem, transitSubsystem));

        NamedCommands.registerCommand("StartIntaking",
                IntakingCommands.startIntakingCommand(shooterSubsystem, intakeSubsystem, transitSubsystem,
                        lightSubsystem));

        NamedCommands.registerCommand("ShootSpeaker",
                ShootingCommands.fullShootSpeakerCommand(shooterSubsystem, transitSubsystem, intakeSubsystem,
                        lightSubsystem));

        // NamedCommands.registerCommand("AutoAlignSpeaker",
        // AlignWithPose.alignWithSpeakerCommand(driveSubsystem));
    }

    private void configureButtonBindings() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            new JoystickButton(driveJoystick, TeleopConstants.RESET_GYRO_BUTTON).onTrue(driveSubsystem.gyroReset());
            new JoystickButton(driveJoystick, TeleopConstants.X_LOCK_BUTTON).onTrue((driveSubsystem.gyroReset()));
            new JoystickButton(driveJoystick, 1).whileTrue(AlignWithPose.alignWithSpeakerCommand(driveSubsystem));
        }

        // Above = DriveJoystick, Below = OperatorJoystick
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            new JoystickButton(operatorJoystick, 2).onTrue(
                    (IntakingCommands.startIntakingCommand(shooterSubsystem, intakeSubsystem, transitSubsystem,
                            lightSubsystem)))
                    .onFalse(IntakingCommands.stopIntakingCommand(shooterSubsystem, intakeSubsystem, transitSubsystem));

            new JoystickButton(operatorJoystick, 7)
                    .onTrue(IntakingCommands.reverseIntakingCommand(shooterSubsystem, intakeSubsystem,
                            transitSubsystem))
                    .onFalse(IntakingCommands.stopIntakingCommand(shooterSubsystem, intakeSubsystem, transitSubsystem));
        }
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            new JoystickButton(operatorJoystick, 4)
                    .onTrue(ShootingCommands.ShootSpeakerChamber(shooterSubsystem, transitSubsystem, intakeSubsystem,
                            lightSubsystem).onlyIf(() -> !UserPolicy.shootCommandLocked))
                    .onFalse(ShootingCommands.ShootSpeaker(shooterSubsystem, transitSubsystem, intakeSubsystem,
                            lightSubsystem).onlyIf(() -> UserPolicy.shootCommandLocked));
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

    public final class UserPolicy {
        public static boolean xLocked = false;
        public static boolean shootCommandLocked = false;
        public static boolean intakeRunning = false;
        public static boolean isManualControlled = true;
    }
}
