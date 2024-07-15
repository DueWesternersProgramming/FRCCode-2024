package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.light.LEDMatch;

public class ChamberAutoCommand extends SequentialCommandGroup {

    public ChamberAutoCommand(ShooterSubsystem shooterSubsystem, TransitSubsystem transitSubsystem,
            IntakeSubsystem intakeSubsystem, LightSubsystem lightsubsystem, int mode) {
        addCommands(
                shooterSubsystem.LockShootCommand(true),
                new LEDMatch(lightsubsystem, 3),
                shooterSubsystem.startShooterCommand(),
                transitSubsystem.reverseTransitCommand(),
                new ReverseIntake(intakeSubsystem),
                new WaitCommand(0.1),
                transitSubsystem.stopTransitCommand(),
                new StopIntake(intakeSubsystem));
    }
}