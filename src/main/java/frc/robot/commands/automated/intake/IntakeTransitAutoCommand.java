package frc.robot.commands.automated.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.light.LEDMatch;

public class IntakeTransitAutoCommand extends SequentialCommandGroup {

    public IntakeTransitAutoCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem, LightSubsystem lightSubsystem) {
        addCommands(
                shooterSubsystem.LockShootCommand(false),
                shooterSubsystem.stopShooterCommand(),
                intakeSubsystem.startIntakeCommand(),
                transitSubsystem.startTransitCommand(),
                new LEDMatch(lightSubsystem, 2));
    }
}