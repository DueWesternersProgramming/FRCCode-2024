package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutomouseTest1 extends SequentialCommandGroup{
    public AutomouseTest1(DriveSubsystem m_drive, IntakeSubsystem m_intake) {
        addCommands(
            new GyroReset(m_drive), 
            new WaitCommand(1),
            new StartIntake(m_intake),
            new StopIntake(m_intake)

            );
    }
}