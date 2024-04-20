package frc.robot.commands.auto.teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.transit.StopTransit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.light.LEDMatch;
import frc.robot.commands.shooter.LockShootCommand;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.transit.ReverseTransit;

public class TransitChamberAutoCommand extends SequentialCommandGroup{
    
    public TransitChamberAutoCommand(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem, IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem, int mode) {
        addCommands(
            new LockShootCommand(true),
            new LEDMatch(lightsubsystem, 3),
            new StartShooter(shooterSubsystem, mode),
            new ReverseTransit(transitSubsystem),
            new ReverseIntake(intakeSubsystem),
            new WaitCommand(0.1),
            new StopTransit(transitSubsystem),
            new StopIntake(intakeSubsystem)
        );
    }
}