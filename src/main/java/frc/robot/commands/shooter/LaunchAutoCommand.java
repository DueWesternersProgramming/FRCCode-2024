package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.ShootIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.light.LEDMatch;

public class LaunchAutoCommand extends SequentialCommandGroup {

    public LaunchAutoCommand(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem,
            IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem, int mode) {
        addCommands(
                new ShootIntake(intakeSubsystem),
                transitSubsystem.startTransitCommand(),
                new WaitCommand(1.5),
                shooterSubsystem.stopShooterCommand(),
                transitSubsystem.stopTransitCommand(),
                new StopIntake(intakeSubsystem),
                shooterSubsystem.LockShootCommand(false),
                new LEDMatch(lightsubsystem, 2));
    }
}