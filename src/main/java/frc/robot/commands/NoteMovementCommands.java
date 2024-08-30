package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.light.LEDMatch;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;

/**
 * This is how the code interacts with the intake and transit subsystems, at a
 * higher level.
 */
public class NoteMovementCommands extends SequentialCommandGroup {

    public NoteMovementCommands() {
    }

    /**
     * This command turns the intake and transit on and stops the shooter.
     */
    public static Command startIntakingCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem, LightSubsystem lightSubsystem) {
        return new SequentialCommandGroup(
                new RunCommand(() -> {
                    System.out.println("Intake and transit on...");
                }),
                shooterSubsystem.LockShootCommand(false),
                shooterSubsystem.stopShooterCommand(),
                intakeSubsystem.startIntakeCommand(),
                transitSubsystem.startTransitCommand(),
                new LEDMatch(lightSubsystem, 2));
    }

    /**
     * This command turns the intake and transit on.
     */
    public static Command reverseIntakingCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem) {
        return new SequentialCommandGroup(
                new RunCommand(() -> {
                    System.out.println("Intake/transit reversed...");
                }),
                shooterSubsystem.stopShooterCommand(),
                intakeSubsystem.reverseIntakeCommand(),
                transitSubsystem.reverseTransitCommand(),
                shooterSubsystem.reverseShooterCommand());
    }

    /**
     * This command stops the intake and transit.
     */
    public static Command stopIntakingCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem) {
        return new SequentialCommandGroup(
                new RunCommand(() -> {
                    System.out.println("All systems stopped...");
                }),
                shooterSubsystem.stopShooterCommand(),
                intakeSubsystem.stopIntakeCommand(),
                transitSubsystem.stopTransitCommand());
    }
}