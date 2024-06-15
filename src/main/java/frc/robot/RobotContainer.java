// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.commands.drive.XCommand;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.light.LEDHasNoteUpdater;
import frc.robot.commands.light.LEDOff;
import frc.robot.commands.light.LEDPrematch;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.transit.StartTransit;
import frc.robot.commands.transit.StopTransit;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.auto.autonomous.OldTransitShootAutoCommand;
import frc.robot.commands.auto.teleop.AutoAlignSpeaker;
import frc.robot.commands.auto.teleop.TransitChamberAutoCommand;
import frc.robot.commands.auto.teleop.TransitLaunchAutoCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.RobotConstants.TeleopConstants;
import frc.robot.commands.auto.IntakeTransitAutoCommand;
import frc.robot.commands.auto.IntakeTransitAutoReverseCommand;
import frc.robot.commands.auto.IntakeTransitAutoStopCommand;
import frc.robot.commands.auto.RobotSystemsCheckCommand;
import frc.robot.commands.drive.GyroReset;

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
            m_autoPositionChooser = AutoBuilder.buildAutoChooser("");
            Shuffleboard.getTab("Autonomous").add(m_autoPositionChooser);
            Shuffleboard.getTab("Power").add(pdp);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void createNamedCommands() {
        NamedCommands.registerCommand("StartShooter",
                new StartShooter(shooterSubsystem, 0));

        NamedCommands.registerCommand("StopShooter",
                new StopShooter(shooterSubsystem));

        NamedCommands.registerCommand("StartIntake",
                new StartIntake(intakeSubsystem));

        NamedCommands.registerCommand("StopIntake",
                new StopIntake(intakeSubsystem));

        NamedCommands.registerCommand("StartTransit", new StartTransit(transitSubsystem));

        NamedCommands.registerCommand("StopTransit",
                new StopTransit(transitSubsystem));

        NamedCommands.registerCommand("IntakeTransitCommand",
                new IntakeTransitAutoCommand(shooterSubsystem, intakeSubsystem, transitSubsystem, lightSubsystem));

        NamedCommands.registerCommand("IntakeTransitStopCommand",
                new IntakeTransitAutoStopCommand(shooterSubsystem, intakeSubsystem, transitSubsystem));

        NamedCommands.registerCommand("IntakeTransit",
                new IntakeTransitAutoCommand(shooterSubsystem, intakeSubsystem, transitSubsystem, lightSubsystem));

        NamedCommands.registerCommand("TransitShootSpeaker",
                new OldTransitShootAutoCommand(shooterSubsystem, transitSubsystem, intakeSubsystem, lightSubsystem, 0));
    }

    private void configureButtonBindings() {
        new JoystickButton(driveJoystick, TeleopConstants.RESET_GYRO_BUTTON).onTrue(new GyroReset(driveSubsystem));
        new JoystickButton(driveJoystick, TeleopConstants.X_LOCK_BUTTON).onTrue((new XCommand()));
        // new JoystickButton(driveJoystick, null).whileTrue(new AutoAlignSpeaker());

        // Above = DriveJoystick, Below = OperatorJoystick

        new JoystickButton(operatorJoystick, 2).onTrue(
                (new IntakeTransitAutoCommand(shooterSubsystem, intakeSubsystem, transitSubsystem, lightSubsystem)))
                .onFalse(new IntakeTransitAutoStopCommand(shooterSubsystem, intakeSubsystem, transitSubsystem));

        new JoystickButton(operatorJoystick, 7)
                .onTrue(new IntakeTransitAutoReverseCommand(shooterSubsystem, intakeSubsystem, transitSubsystem))
                .onFalse(new IntakeTransitAutoStopCommand(shooterSubsystem, intakeSubsystem, transitSubsystem));

        new JoystickButton(operatorJoystick, 4)
                .onTrue(new TransitChamberAutoCommand(shooterSubsystem, transitSubsystem, intakeSubsystem,
                        lightSubsystem, 0).onlyIf(() -> !UserPolicy.shootCommandLocked))
                .onFalse(new TransitLaunchAutoCommand(shooterSubsystem, transitSubsystem, intakeSubsystem,
                        lightSubsystem, 0).onlyIf(() -> UserPolicy.shootCommandLocked)); // SPEAKER
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
            return new GyroReset(driveSubsystem);
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
    }
}
