package frc.robot.commands.automated.shoot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.light.LEDMatch;

public class OldTransitShootAutoCommand extends SequentialCommandGroup {

    public OldTransitShootAutoCommand(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem,
            IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem, int mode) {
        addCommands(
                shooterSubsystem.LockShootCommand(true),
                new LEDMatch(lightsubsystem, 3),
                shooterSubsystem.startShooterCommand(),
                transitSubsystem.reverseTransitCommand(),
                new WaitCommand(0.25),
                transitSubsystem.stopTransitCommand(),
                new WaitCommand(0.2),
                intakeSubsystem.startIntakeCommand(),
                transitSubsystem.startTransitCommand(),
                new WaitCommand(1),
                shooterSubsystem.stopShooterCommand(),
                transitSubsystem.stopTransitCommand(),
                intakeSubsystem.stopIntakeCommand(),
                new LEDMatch(lightsubsystem, 2),
                shooterSubsystem.LockShootCommand(false));
    }
}