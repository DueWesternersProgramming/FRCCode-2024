package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;

public class ClimberSubsystem extends SubsystemBase {

    CANSparkMax climberMotor1, climberMotor2;
    RelativeEncoder climber1RelativeEncoder, climber2RelativeEncoder;

    public ClimberSubsystem() {
        if (SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED && RobotBase.isReal()) {
            climberMotor1 = new CANSparkMax(PortConstants.CAN.LEFT_CLIMBER_PORT, MotorType.kBrushless);
            climberMotor2 = new CANSparkMax(PortConstants.CAN.RIGHT_CLIMBER_PORT, MotorType.kBrushless);
            climberMotor1.setIdleMode(IdleMode.kBrake);
            climberMotor2.setIdleMode(IdleMode.kBrake);
            climberMotor1.setInverted(true);
            climber1RelativeEncoder = climberMotor1.getEncoder();
            climber2RelativeEncoder = climberMotor2.getEncoder();
            resetEncoders();
        }
    }

    public double getSpeed() {
        return SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED && RobotBase.isReal() ? climberMotor1.get() : 0;
    }

    public void setSpeed(double leftSpeed, double rightSpeed, boolean override) {
        if (SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED && RobotBase.isReal()) {
            if ((leftSpeed < 0.1 && leftSpeed > -0.1) || (rightSpeed < 0.1 && rightSpeed > -0.1)) {
                if ((getEncoder1Position() > 0 && leftSpeed > 0 && !override)) {
                    climberMotor1.set(0);
                } else {
                    climberMotor1.set(leftSpeed);

                }
                if ((getEncoder2Position() > 0 && leftSpeed > 0 && !override)) {
                    climberMotor2.set(0);
                } else {
                    if (override) {
                        climberMotor2.set(rightSpeed);
                    } else {
                        climberMotor2.set(leftSpeed);
                    }
                }
            }
        }
    }

    public double getEncoder1Position() {
        return SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED && RobotBase.isReal()
                ? climber1RelativeEncoder.getPosition()
                : 0;
    }

    public double getEncoder2Position() {
        return SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED && RobotBase.isReal()
                ? climber2RelativeEncoder.getPosition()
                : 0;
    }

    public void resetEncoders() {
        if (SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED && RobotBase.isReal()) {
            climber1RelativeEncoder.setPosition(0.0);
            climber2RelativeEncoder.setPosition(0.0);
        }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.CLIMBER_SUBSYSTEM_ENABLED) {

        }
    }
}