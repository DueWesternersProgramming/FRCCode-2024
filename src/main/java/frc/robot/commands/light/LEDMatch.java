// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.light;

import frc.robot.subsystems.LightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LEDMatch extends Command {
  public final LightSubsystem m_lightSubsystem;
  private int m_itemRequest = 0;

  /**
   * Creates a new TankDrive command.
   *
   * @param lightSubsystem The subsystem used by this command.
   * @param itemRequest 0 = nothing (red), 1 = cone (white), 2 = cube (blue)
   */
  public LEDMatch(LightSubsystem lightSubsystem, int itemRequest) {
    m_lightSubsystem = lightSubsystem; 
    m_itemRequest = itemRequest;
    addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lightSubsystem.stopAnimation(0);
    m_lightSubsystem.stopAnimation(1);
    m_lightSubsystem.stopAnimation(2);
    if (m_itemRequest == 0) {
      m_lightSubsystem.setColor(255, 0, 0);
    }
    else if (m_itemRequest == 1){
      m_lightSubsystem.setColor(255, 230, 0);
    }
    else if (m_itemRequest == 2) {
      m_lightSubsystem.setColor(150, 27, 191);
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