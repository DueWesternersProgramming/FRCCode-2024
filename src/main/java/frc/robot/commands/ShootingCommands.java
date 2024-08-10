package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.light.LEDMatch;

public class ShootingCommands extends SequentialCommandGroup {

    public ShootingCommands() {
    }

    /**
     * This command is the 2nd and 3rd step of pullback-> rev-> launch (feed).
     */
    public static Command ShootSpeaker(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem,
            IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem) {
        return new SequentialCommandGroup(
                intakeSubsystem.shootIntakeCommand(),
                transitSubsystem.startTransitCommand(),
                new WaitCommand(1.5),
                shooterSubsystem.stopShooterCommand(),
                transitSubsystem.stopTransitCommand(),
                intakeSubsystem.stopIntakeCommand(),
                shooterSubsystem.LockShootCommand(false),
                new LEDMatch(lightsubsystem, 2));
    }

    /**
     * This command is the first step of pullback-> rev-> launch.
     */
    public static Command ShootSpeakerChamber(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem,
            IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem) {
        return new SequentialCommandGroup(
                shooterSubsystem.LockShootCommand(true),
                new LEDMatch(lightsubsystem, 3),
                shooterSubsystem.startShooterCommand(),
                transitSubsystem.reverseTransitCommand(),
                intakeSubsystem.reverseIntakeCommand(),
                new WaitCommand(0.1),
                transitSubsystem.stopTransitCommand(),
                intakeSubsystem.stopIntakeCommand());
    }

    /**
     * This command should only be used for autos. It is a tuned, fast chamber and
     * shoot combination to perform one motion of pullback-> rev-> launch.
     */
    public static Command fullShootSpeakerCommand(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem,
            IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem) {
        return new SequentialCommandGroup(
                new RunCommand(() -> {
                    System.out.println("Shooting speaker...");
                }),
                shooterSubsystem.LockShootCommand(true),
                new LEDMatch(lightsubsystem, 3),
                shooterSubsystem.startShooterCommand(),
                transitSubsystem.reverseTransitCommand(),
                new WaitCommand(0.25),
                transitSubsystem.stopTransitCommand(),
                new WaitCommand(0.2),
                intakeSubsystem.startIntakeCommand(),
                transitSubsystem.startTransitCommand(),
                new WaitCommand(1),
                shooterSubsystem.stopShooterCommand(),
                transitSubsystem.stopTransitCommand(),
                intakeSubsystem.stopIntakeCommand(),
                new LEDMatch(lightsubsystem, 2),
                shooterSubsystem.LockShootCommand(false));
    }

}