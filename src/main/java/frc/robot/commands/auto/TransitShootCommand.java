package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.transit.StartTransit;
import frc.robot.commands.transit.StopTransit;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;

public class TransitShootCommand extends SequentialCommandGroup{
    
    public TransitShootCommand(ShooterSubsystem shooter_Subsystem, TransitSubsystem transit_Subsystem) {
        addCommands(
            new StartShooter(shooter_Subsystem),
            new WaitCommand(1),
            new StartTransit(transit_Subsystem),
            new WaitCommand(2),
            new StopShooter(shooter_Subsystem),
            new StopTransit(transit_Subsystem)

        );
    }
}