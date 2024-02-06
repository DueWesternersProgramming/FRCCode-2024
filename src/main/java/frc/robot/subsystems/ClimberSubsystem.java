package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase{
    
    CANSparkMax climberMotor1 = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_1_PORT, MotorType.kBrushless);
    CANSparkMax climberMotor2 = new CANSparkMax(ClimberConstants.CLIMBER_MOTOR_2_PORT, MotorType.kBrushless);
    RelativeEncoder climber1RelativeEncoder = climberMotor1.getEncoder();
    RelativeEncoder climber2RelativeEncoder = climberMotor2.getEncoder();

    public ClimberSubsystem(){
        climberMotor1.setIdleMode(IdleMode.kBrake);
        climberMotor2.setIdleMode(IdleMode.kBrake);
        resetEncoders();
    }

    public double getSpeed() {
        return climberMotor1.get();
    }

    public void setSpeed(double speed) {
        climberMotor1.set(speed);
        climberMotor2.set(speed);
    }

    public double getEncoder1Position() {
        return climber1RelativeEncoder.getPosition();
    }

    public double getEncoder2Position() {
        return climber2RelativeEncoder.getPosition();
    }

    public void resetEncoders() {
        climber1RelativeEncoder.setPosition(0.0);
        climber2RelativeEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", getSpeed());
    }
}