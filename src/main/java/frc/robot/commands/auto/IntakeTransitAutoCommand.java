package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.transit.StartTransit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.light.LEDMatch;
import frc.robot.commands.shooter.LockShootCommand;
import frc.robot.commands.shooter.StopShooter;

public class IntakeTransitAutoCommand extends SequentialCommandGroup{
    
    public IntakeTransitAutoCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, TransitSubsystem transitSubsystem, LightSubsystem lightSubsystem) {
        addCommands(
            new LockShootCommand(false),
            new StopShooter(shooterSubsystem),
            new StartIntake(intakeSubsystem),
            new StartTransit(transitSubsystem),
            new LEDMatch(lightSubsystem, 2)
        );
    }
}