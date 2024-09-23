package frc.robot.commands.automated;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.transit.ReverseTransit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.shooter.ReverseShooter;
import frc.robot.commands.shooter.StopShooter;

public class IntakeTransitAutoReverseCommand extends SequentialCommandGroup {

    public IntakeTransitAutoReverseCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem) {
        addCommands(
                new StopShooter(shooterSubsystem),
                new ReverseIntake(intakeSubsystem),
                new ReverseTransit(transitSubsystem),
                new ReverseShooter(shooterSubsystem));
    }
}