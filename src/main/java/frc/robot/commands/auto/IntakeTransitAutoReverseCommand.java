package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.transit.ReverseTransit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.shooter.ReverseShooter;
import frc.robot.commands.shooter.StopShooter;

public class IntakeTransitAutoReverseCommand extends SequentialCommandGroup{
    
    public IntakeTransitAutoReverseCommand(ShooterSubsystem shooter_Subsystem, IntakeSubsystem intake_Subsystem, TransitSubsystem transit_Subsystem) {
        addCommands(
            new StopShooter(shooter_Subsystem),
            new ReverseIntake(intake_Subsystem),
            new ReverseTransit(transit_Subsystem),
            new ReverseShooter(shooter_Subsystem)
        );
    }
}