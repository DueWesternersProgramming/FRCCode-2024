// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.GyroReset;
import frc.robot.commands.drive.TwistCommand;
import frc.robot.commands.drive.XCommand;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.transit.StartTransit;
import frc.robot.commands.transit.StopTransit;
import frc.robot.commands.camera.AutoAimCommand;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


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

    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.CONTROLLER.DRIVE_JOYSTICK);
    private final Joystick operatorJoystick = new Joystick(RobotConstants.PortConstants.CONTROLLER.OPERATOR_JOYSTICK);

    SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

    PowerDistribution PDP = new PowerDistribution(16, ModuleType.kRev);

    private final Field2d field = new Field2d();

    public RobotContainer() {
        System.out.println("HI");
        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driveJoystick));
        climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem, operatorJoystick));
        shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem, operatorJoystick));
        
        configureButtonBindings();
        //AutoBuilder.buildAuto("lineAuto");

        m_autoPositionChooser = AutoBuilder.buildAutoChooser("lineAuto");
        Shuffleboard.getTab("Autonomous").add(m_autoPositionChooser);
        Shuffleboard.getTab("Power").add(PDP);
    }

    private void configureButtonBindings(){
        System.out.println("HI2");
        new JoystickButton(driveJoystick, 1).whileTrue(new TwistCommand());
        new JoystickButton(driveJoystick,11).whileTrue(new GyroReset(driveSubsystem));
        new JoystickButton(driveJoystick, 3).whileTrue((new XCommand()));
        new JoystickButton(driveJoystick, 7).whileTrue(new AutoAimCommand(driveSubsystem, visionSubsystem));
        
        new JoystickButton(operatorJoystick, 3).onTrue((new StartIntake(intakeSubsystem)));
        new JoystickButton(operatorJoystick, 1).onTrue((new StopIntake(intakeSubsystem)));
        
        new JoystickButton(operatorJoystick, 2).whileTrue((new StartShooter(shooterSubsystem))).whileFalse(new StopShooter(shooterSubsystem));
        //new JoystickButton(operatorJoystick, 1).onTrue((new StopShooter(shooterSubsystem)));

        new JoystickButton(operatorJoystick, 4).onTrue((new StartTransit(transitSubsystem)));
        new JoystickButton(operatorJoystick, 5).onTrue((new StopTransit(transitSubsystem)));
    }

    public Command getAutonomousCommand() {
        return m_autoPositionChooser.getSelected();
    }

    public Field2d getField() {
        return field;
    }
    
    public final class UserPolicy {
        public static boolean twistable = false;
        public static boolean xLocked = false;
    }


    //     public TrajectoryConfig createTrajectoryConfig() {
    //     TrajectoryConfig config = new TrajectoryConfig(
    //             RobotConstants.AutonomousConstants.MAX_SPEED_METERS_PER_SECOND,
    //             RobotConstants.AutonomousConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
    //             .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

    
    //     return config;
    // }

    // public Trajectory createExampleTrajectory(TrajectoryConfig config) {
    //     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //             new Pose2d(0, 0, new Rotation2d(0)),
    //             List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //             new Pose2d(3, 0, new Rotation2d(0)),
    //             config);

    //     return exampleTrajectory;
    // }
}
