// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.light;

import frc.robot.subsystems.LightSubsystem;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LEDMatch extends Command {
  public final LightSubsystem m_lightSubsystem;
  private int m_mode = 0;

  /**
   * Creates a new TankDrive command.
   *
   * @param lightSubsystem The subsystem used by this command.
   * @param mode 0 = auto (red), 1 = teleop (green), 2 = visiontarget (rainbow), 3 = shooting (cool animation)
   */
  public LEDMatch(LightSubsystem lightSubsystem, int mode) {
    m_lightSubsystem = lightSubsystem; 
    m_mode = mode;
    addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lightSubsystem.stopAnimation(0);
    m_lightSubsystem.stopAnimation(1);
    switch (m_mode){
      case 0:
        m_lightSubsystem.setColor(255, 0, 0);
        break;
      case 1:
        m_lightSubsystem.setColor(0, 255, 0);
        break;
      case 2:
        m_lightSubsystem.setAnimation(new ColorFlowAnimation(0, 0, 255, 0, 0.3, 250, Direction.Forward, 8), 0);
        m_lightSubsystem.setAnimation(new ColorFlowAnimation(255, 0, 0, 0, 0.3, 250, Direction.Backward, 8), 1);
        break;
      case 3:
        m_lightSubsystem.setAnimation(new ColorFlowAnimation(255, 0, 0, 0, 1, 250, Direction.Forward, 8), 0);
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