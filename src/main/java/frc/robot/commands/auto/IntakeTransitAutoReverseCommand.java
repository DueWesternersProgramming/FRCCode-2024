package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.ReverseIntake;

public class IntakeTransitAutoReverseCommand extends SequentialCommandGroup {

    public IntakeTransitAutoReverseCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem) {
        addCommands(
                shooterSubsystem.stopShooterCommand(),
                new ReverseIntake(intakeSubsystem),
                transitSubsystem.reverseTransitCommand(),
                shooterSubsystem.reverseShooterCommand());
    }
}