package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.StopIntake;

public class IntakeTransitAutoStopCommand extends SequentialCommandGroup {

    public IntakeTransitAutoStopCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem) {
        addCommands(
                // Stop all
                shooterSubsystem.stopShooterCommand(),
                new StopIntake(intakeSubsystem),
                transitSubsystem.stopTransitCommand());
    }
}