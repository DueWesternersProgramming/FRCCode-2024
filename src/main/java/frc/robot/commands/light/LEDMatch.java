// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.light;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;

import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LEDMatch extends Command {
  private final LightSubsystem lightSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private int m_mode = 0;

  /**
   * Creates a new TankDrive command.
   *
   * @param lightSubsystem The subsystem used by this command.
   * @param mode 0 = red, 1 = green, 2 = noteDetector, 3 = shooting (cool animation)
   */
  public LEDMatch(LightSubsystem lightSubsystem, IntakeSubsystem intakeSubsystem, int mode) {
    this.lightSubsystem = lightSubsystem; 
    this.intakeSubsystem = intakeSubsystem;
    m_mode = mode;
    addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightSubsystem.stopAnimation(0);
    lightSubsystem.stopAnimation(1);
    switch (m_mode){
      case 0:
        lightSubsystem.setColor(255, 0, 0, 0, 9, 150);
        break;
      case 1:
        lightSubsystem.setColor(0, 255, 0, 0, 9, 150);
        break;
      case 2:
        new LEDHasNoteUpdater(lightSubsystem, intakeSubsystem).schedule();
        break;
      case 3:
        lightSubsystem.setColor(0, 0, 0);
        lightSubsystem.setAnimation(new LarsonAnimation(0, 0, 255, 0, 0.35, 24, BounceMode.Front, 5, 25), 0);
        lightSubsystem.setAnimation(new LarsonAnimation(0, 0, 255, 0, 0.35, 24, BounceMode.Front, 5, 102), 1);
        break;
    }
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}