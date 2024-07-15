package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;

public class IntakeTransitAutoStopCommand extends SequentialCommandGroup {

    public IntakeTransitAutoStopCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem) {
        addCommands(
                // Stop all
                shooterSubsystem.stopShooterCommand(),
                intakeSubsystem.stopIntakeCommand(),
                transitSubsystem.stopTransitCommand());
    }
}