// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.light;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;

import java.time.LocalTime;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class LEDHasNoteUpdater extends Command {
  public final LightSubsystem lightSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public double previousTime = 0;

  /**
   * Creates a new LEDHasNote command.
   *
   * @param lightSubsystem The subsystem used by this command.
   * @param intakeSubsystem The second subsystem used by this command.
   */
  public LEDHasNoteUpdater(LightSubsystem lightSubsystem, IntakeSubsystem intakeSubsystem) {
    this.lightSubsystem = lightSubsystem; 
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        lightSubsystem.stopAnimation(0);
        lightSubsystem.stopAnimation(1);
    }
  
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (previousTime < (LocalTime.now().toSecondOfDay() - 1)){
      if (intakeSubsystem.seesNote()) {
          lightSubsystem.setColor(0, 255, 0);
          previousTime = LocalTime.now().toSecondOfDay();
      }
      else {
          lightSubsystem.setColor(255, 0, 0);
      }
    }
    new WaitCommand(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}