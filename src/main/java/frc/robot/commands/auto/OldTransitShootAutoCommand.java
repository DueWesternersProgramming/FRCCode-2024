package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.transit.StartTransit;
import frc.robot.commands.transit.StopTransit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.light.LEDMatch;
import frc.robot.commands.shooter.LockShootCommand;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.transit.ReverseTransit;

public class OldTransitShootAutoCommand extends SequentialCommandGroup{
    
    public OldTransitShootAutoCommand(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem, IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem, int mode) {
        addCommands(
            new LockShootCommand(true),
            new LEDMatch(lightsubsystem, 3),
            new StartShooter(shooterSubsystem, mode),
            new ReverseTransit(transitSubsystem),
            new WaitCommand(0.25),
            new StopTransit(transitSubsystem),
            new WaitCommand(0.2),
            new StartIntake(intakeSubsystem),
            new StartTransit(transitSubsystem),
            new WaitCommand(1),
            new StopShooter(shooterSubsystem),
            new StopTransit(transitSubsystem),
            new StopIntake(intakeSubsystem),
            new LEDMatch(lightsubsystem, 2),
            new LockShootCommand(false)
        );
    }
}