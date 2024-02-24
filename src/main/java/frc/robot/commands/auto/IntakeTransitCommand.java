package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.transit.StartTransit;
import frc.robot.commands.transit.StopTransit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;

public class IntakeTransitCommand extends SequentialCommandGroup{
    
    public IntakeTransitCommand(ShooterSubsystem shooter_Subsystem, IntakeSubsystem intake_Subsystem, TransitSubsystem transit_Subsystem) {
        addCommands(
            new StopShooter(shooter_Subsystem),
            new StartIntake(intake_Subsystem),
            new StartTransit(transit_Subsystem)
        );
    }
}