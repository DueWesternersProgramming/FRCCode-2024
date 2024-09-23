package frc.robot.commands.automated;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.MoveAtPowerCommand;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.transit.StartTransit;
import frc.robot.commands.transit.StopTransit;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;

public class RobotSystemsCheckCommand extends SequentialCommandGroup {
    public RobotSystemsCheckCommand(DriveSubsystem drivesubsystem, ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem, TransitSubsystem transitSubsystem) {
        addCommands(
                new PrintCommand("Starting Robot Systems Checks!"),
                new WaitCommand(2),
                new PrintCommand("Make Sure The Robot Is Not On The Ground And Motors Are Clear..."),
                new WaitCommand(5),
                new PrintCommand("Starting...\nTesting Shooter..."),
                new StartShooter(shooterSubsystem, 1),
                new WaitCommand(3),
                new StartShooter(shooterSubsystem, 0),
                new WaitCommand(3),
                new StopShooter(shooterSubsystem),
                new PrintCommand("Shooter Complete!\nTesting Transit..."),
                new StartTransit(transitSubsystem),
                new WaitCommand(3),
                new StopTransit(transitSubsystem),
                new PrintCommand("Transit Complete!\nTesting Intake..."),
                new StartIntake(intakeSubsystem),
                new WaitCommand(3),
                new StopIntake(intakeSubsystem),
                new PrintCommand("Intake Complete!\nTesting Drive..."),
                new MoveAtPowerCommand(drivesubsystem, 0, -0.5, 0),
                new WaitCommand(1),
                new MoveAtPowerCommand(drivesubsystem, 0.5, 0, 0),
                new WaitCommand(1),
                new MoveAtPowerCommand(drivesubsystem, 0, 0.5, 0),
                new WaitCommand(1),
                new MoveAtPowerCommand(drivesubsystem, -0.5, 0, 0),
                new WaitCommand(1),
                new MoveAtPowerCommand(drivesubsystem, 0, 0, 0.5),
                new WaitCommand(1),
                new MoveAtPowerCommand(drivesubsystem, 0, 0, -0.5),
                new WaitCommand(1),
                new MoveAtPowerCommand(drivesubsystem, 0, 0, 0),
                new PrintCommand("Testing Complete!"));
    }
}
