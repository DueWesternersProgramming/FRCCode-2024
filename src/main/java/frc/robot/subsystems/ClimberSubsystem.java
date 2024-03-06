package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;


public class ClimberSubsystem extends SubsystemBase{
    
    CANSparkMax climberMotor1;
    CANSparkMax climberMotor2;
    RelativeEncoder climber1RelativeEncoder;
    RelativeEncoder climber2RelativeEncoder;

    public ClimberSubsystem(){
        if (SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED){
            climberMotor1 = new CANSparkMax(PortConstants.CAN.LEFT_CLIMBER_PORT, MotorType.kBrushless);
            climberMotor2 = new CANSparkMax(PortConstants.CAN.RIGHT_CLIMBER_PORT, MotorType.kBrushless);
            climberMotor1.setIdleMode(IdleMode.kBrake);
            climberMotor2.setIdleMode(IdleMode.kBrake);
            climberMotor1.setInverted(true);
            climberMotor1.burnFlash();
            climberMotor2.burnFlash();
            climber1RelativeEncoder = climberMotor1.getEncoder();
            climber2RelativeEncoder = climberMotor2.getEncoder();
            resetEncoders();
        }
    }

    public double getSpeed() {
        return SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED ? climberMotor1.get() : 0;
    }

    public void setSpeed(double speed) {
        if (SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED){
            if (getEncoder1Position() > 0 && speed > 0){
                climberMotor1.set(0);
            }
            else {
                climberMotor1.set(speed);
            }
            if (getEncoder2Position() > 0 && speed > 0){
                climberMotor2.set(0);
            }
            else {
                climberMotor2.set(speed);
            }
        }
    }

    public double getEncoder1Position() {
        return SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED ? climber1RelativeEncoder.getPosition() : 0;
    }

    public double getEncoder2Position() {
        return SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED ? climber2RelativeEncoder.getPosition() : 0;
    }

    public void resetEncoders() {
        if (SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED){
            climber1RelativeEncoder.setPosition(0.0);
            climber2RelativeEncoder.setPosition(0.0);
        }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED){
            SmartDashboard.putNumber("Climber Speed", getSpeed());
            SmartDashboard.putNumber("Climber L Position", getEncoder1Position());
            SmartDashboard.putNumber("Climber R Position", getEncoder2Position());
        }
    }
}