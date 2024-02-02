// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GyroReset;
import frc.robot.commands.TwistCommand;
import frc.robot.commands.XCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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

    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.CONTROLLER.JOYSTICK);

    SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

    PowerDistribution PDP = new PowerDistribution(16, ModuleType.kRev);

    private final Field2d field = new Field2d(); // a representation of the field

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driveJoystick));
        
        configureButtonBindings();

        Shuffleboard.getTab("Autonomous").add(m_autoPositionChooser);
        Shuffleboard.getTab("Power").add(PDP);
    }

    private void configureButtonBindings(){
        new JoystickButton(driveJoystick, 1).whileTrue(new TwistCommand());
        new JoystickButton(driveJoystick,11).whileTrue(new GyroReset(driveSubsystem));
        new JoystickButton(driveJoystick, 3).whileTrue((new XCommand()));

        new JoystickButton(driveJoystick, 7).whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        new JoystickButton(driveJoystick, 8).whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        new JoystickButton(driveJoystick, 9).whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        new JoystickButton(driveJoystick, 10).whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
