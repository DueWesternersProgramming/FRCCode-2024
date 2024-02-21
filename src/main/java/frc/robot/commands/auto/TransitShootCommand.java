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
    
    public TransitShootCommand(ShooterSubsystem shooter, TransitSubsystem transit) {
        addCommands(
            new StartShooter(shooter),
            new WaitCommand(0.25),
            new StartTransit(transit),
            new WaitCommand(0.2),
            new StopShooter(shooter),
            new StopTransit(transit)

        );
    }
}