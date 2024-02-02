// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.light;

import frc.robot.subsystems.LightSubsystem;

import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LEDPit extends Command {
  public final LightSubsystem m_lightSubsystem;
  private LarsonAnimation larsonAnimation;


  /**
   * Creates a new TankDrive command.
   *
   * @param lightSubsystem The subsystem used by this command.
   */
  public LEDPit(LightSubsystem lightSubsystem) {
    m_lightSubsystem = lightSubsystem; 
    larsonAnimation = new LarsonAnimation(0, 250, 0, 0, 0.5, 230, BounceMode.Back, 7, 7);
    addRequirements(lightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  //m_lightSubsystem.setIdleMode{
  public void initialize() {
    m_lightSubsystem.setAnimation(larsonAnimation,0);
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