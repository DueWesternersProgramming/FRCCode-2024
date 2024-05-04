package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.MoveAtPowerCommand;
import frc.robot.subsystems.DriveSubsystem;


public class RobotSystemsCheckCommand extends SequentialCommandGroup{
    public RobotSystemsCheckCommand(DriveSubsystem drivesubsystem) {
        addCommands(
            new PrintCommand("Starting Robot Systems Checks!"),
            new WaitCommand(2),
            new PrintCommand("Make Sure The Robot Is Not On The Ground And Motors Are Clear..."),
            new WaitCommand(3),
            //Add commands (per robot) for a complete system check off ALL subsystems of the robot
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
            new PrintCommand("Testing Complete!")
        );
    }
}
