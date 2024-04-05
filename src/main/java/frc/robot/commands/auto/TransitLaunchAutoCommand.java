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
import frc.robot.commands.shooter.StopShooter;

public class TransitLaunchAutoCommand extends SequentialCommandGroup{
    
    public TransitLaunchAutoCommand(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem, IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem, int mode) {
        addCommands(
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