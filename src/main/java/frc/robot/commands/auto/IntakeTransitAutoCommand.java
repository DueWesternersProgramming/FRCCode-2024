package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.light.LEDMatch;

public class IntakeTransitAutoCommand extends SequentialCommandGroup {

    public IntakeTransitAutoCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            TransitSubsystem transitSubsystem, LightSubsystem lightSubsystem) {
        addCommands(
                shooterSubsystem.LockShootCommand(false),
                shooterSubsystem.stopShooterCommand(),
                new StartIntake(intakeSubsystem),
                transitSubsystem.startTransitCommand(),
                new LEDMatch(lightSubsystem, 2));
    }
}