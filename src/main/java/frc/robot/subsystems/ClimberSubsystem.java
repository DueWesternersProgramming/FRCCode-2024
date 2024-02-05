package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase{
    
    private static final CANSparkBase climberMotor = null;
    CANSparkMax climber = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_PORT_1, MotorType.kBrushless);
    RelativeEncoder climberRelativeEncoder = climberMotor.getEncoder();

    public ClimberSubsystem(){
        climberMotor.setIdleMode(IdleMode.kBrake);
        resetEncoder();
    }

    public double getSpeed() {
        return climberMotor.get();
    }

    public void climberUp(){
        public double getEncoderPosition()
        climberMotor.set(ClimberConstants.CLIMBER_MOTOR_SPEED);
    }

    public void climberReverse(){
        climberMotor.set(ClimberConstants.CLIMBER_MOTOR_REVERSE_SPEED);
    }

    public void climberDown(){
        climberMotor.set(0);
    }

    public double getEncoderPosition() {
        return climberRelativeEncoder.getPosition();
    }

    public void resetEncoder() {
        climberRelativeEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", getSpeed());
    }
}