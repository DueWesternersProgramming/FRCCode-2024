package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;

public class TransitShootAutoCommand extends SequentialCommandGroup{
    
    public TransitShootAutoCommand(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem, IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem, int mode) {
        addCommands(
            new TransitChamberAutoCommand(shooterSubsystem, transitSubsystem, intakeSubsystem, lightsubsystem, mode),
            new WaitCommand(0.2),
            new TransitLaunchAutoCommand(shooterSubsystem, transitSubsystem, intakeSubsystem, lightsubsystem, mode)
        );
    }
}