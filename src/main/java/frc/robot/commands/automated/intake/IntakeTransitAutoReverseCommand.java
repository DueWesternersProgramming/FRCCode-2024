package frc.robot.commands.automated.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;

public class IntakeTransitAutoReverseCommand extends SequentialCommandGroup {

    public IntakeTransitAutoReverseCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem) {
        addCommands(
                shooterSubsystem.stopShooterCommand(),
                intakeSubsystem.reverseIntakeCommand(),
                transitSubsystem.reverseTransitCommand(),
                shooterSubsystem.reverseShooterCommand());
    }
}