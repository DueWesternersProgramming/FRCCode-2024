package frc.robot.commands.auto;

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

public class RobotSystemsCheckCommand extends SequentialCommandGroup{
    public RobotSystemsCheckCommand(DriveSubsystem drive_subsystem, ShooterSubsystem shooter_Subsystem, IntakeSubsystem intake_Subsystem, TransitSubsystem transit_Subsystem) {
        addCommands(
            new PrintCommand("Starting Robot Systems Checks!"),
            new WaitCommand(2),
            new PrintCommand("Make Sure The Robot Is Not On The Ground And Motors Are Clear..."),
            new WaitCommand(5),
            new PrintCommand("Starting...\nTesting Shooter..."),
            new StartShooter(shooter_Subsystem, 1),
            new WaitCommand(3),
            new StartShooter(shooter_Subsystem, 0),
            new WaitCommand(3),
            new StopShooter(shooter_Subsystem),
            new PrintCommand("Shooter Complete!\nTesting Transit..."),
            new StartTransit(transit_Subsystem),
            new WaitCommand(3),
            new StopTransit(transit_Subsystem),
            new PrintCommand("Transit Complete!\nTesting Intake..."),
            new StartIntake(intake_Subsystem),
            new WaitCommand(3),
            new StopIntake(intake_Subsystem),
            new PrintCommand("Intake Complete!\nTesting Drive..."),
            new MoveAtPowerCommand(drive_subsystem, 0, -0.5, 0),
            new WaitCommand(1),
            new MoveAtPowerCommand(drive_subsystem, 0.5, 0, 0),
            new WaitCommand(1),
            new MoveAtPowerCommand(drive_subsystem, 0, 0.5, 0),
            new WaitCommand(1),
            new MoveAtPowerCommand(drive_subsystem, -0.5, 0, 0),
            new WaitCommand(1),
            new MoveAtPowerCommand(drive_subsystem, 0, 0, 0.5),
            new WaitCommand(1),
            new MoveAtPowerCommand(drive_subsystem, 0, 0, -0.5),
            new WaitCommand(1),
            new MoveAtPowerCommand(drive_subsystem, 0, 0, 0),
            new PrintCommand("Testing Complete!")
        );
    }
}
