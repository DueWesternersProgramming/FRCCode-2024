package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.transit.StartTransit;
import frc.robot.commands.transit.StopTransit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.transit.ReverseTransit;

public class TransitShootAutoCommand extends SequentialCommandGroup{
    
    public TransitShootAutoCommand(ShooterSubsystem shooter_Subsystem, TransitSubsystem transit_Subsystem, IntakeSubsystem intake_Subsystem, int mode) {
        addCommands(
            new ReverseTransit(transit_Subsystem),
            new WaitCommand(0.25),
            new StopTransit(transit_Subsystem),
            new StartShooter(shooter_Subsystem, mode),
            new WaitCommand(0.5),
            new StartTransit(transit_Subsystem),
            new StartIntake(intake_Subsystem),
            new WaitCommand(1),
            new StopShooter(shooter_Subsystem),
            new StopTransit(transit_Subsystem),
            new StopIntake(intake_Subsystem)
        );
    }
}