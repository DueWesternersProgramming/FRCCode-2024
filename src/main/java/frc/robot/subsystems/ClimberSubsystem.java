package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.RobotConstants.PortConstants;

public class ClimberSubsystem extends SubsystemBase{
    
    CANSparkMax leftClimberMotor = new CANSparkMax(PortConstants.CAN.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax rightClimberMotor = new CANSparkMax(PortConstants.CAN.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder leftClimberEncoder = leftClimberMotor.getEncoder();
    RelativeEncoder rightClimbEncoder = leftClimberMotor.getEncoder();

    public ClimberSubsystem(){
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
        resetEncoder();
    }

    public double getSpeed() {
        return leftClimberMotor.get();
    }


    public double getLeftEncoderPosition() {
        return leftClimberEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return rightClimbEncoder.getPosition();
    }

    public void resetEncoder() {
        leftClimberEncoder.setPosition(0.0);
        rightClimbEncoder.setPosition(0.0);
    }
    public void brakeMode(){
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        
    }
}