package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.transit.StopTransit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StopShooter;

public class IntakeTransitAutoStopCommand extends SequentialCommandGroup{
    
    public IntakeTransitAutoStopCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, TransitSubsystem transitSubsystem) {
        addCommands(
            new StopShooter(shooterSubsystem),
            new StopIntake(intakeSubsystem),
            new StopTransit(transitSubsystem)
        );
    }
}