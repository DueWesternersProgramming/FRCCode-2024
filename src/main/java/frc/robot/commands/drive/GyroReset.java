package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

public class GyroReset extends Command {
    private final Runnable reset;

    public GyroReset(DriveSubsystem driveSubsystem) {
        reset = () -> {
            driveSubsystem.zeroHeading();
        };
    }

    @Override
    public void initialize() {
        reset.run();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
