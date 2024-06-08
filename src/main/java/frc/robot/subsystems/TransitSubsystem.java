package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.TransitConstants;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;

public class TransitSubsystem extends SubsystemBase {
    CANSparkMax transitMotor1, transitMotor2;
    RelativeEncoder transitEncoder1, transitEncoder2;

    public TransitSubsystem() {
        if (SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED) {
            transitMotor1 = new CANSparkMax(PortConstants.CAN.TRANSIT_MOTOR_PORT_1, MotorType.kBrushless);
            transitMotor2 = new CANSparkMax(PortConstants.CAN.TRANSIT_MOTOR_PORT_2, MotorType.kBrushless);
            transitMotor1.setIdleMode(IdleMode.kBrake);
            transitMotor2.setIdleMode(IdleMode.kBrake);
            transitEncoder1 = transitMotor1.getEncoder();
            transitEncoder2 = transitMotor2.getEncoder();
            // transitMotor1.burnFlash();
            // transitMotor2.burnFlash();
            resetEncoder();
        }
    }

    public double getspeed1() {
        return SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED ? transitMotor1.get() : 0;
    }

    public double getspeed2() {
        return SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED ? transitMotor2.get() : 0;
    }

    public void transitOn() {
        if (SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED) {
            transitMotor1.set(-TransitConstants.TRANSIT_MOTOR_SPEED);
            transitMotor2.set(TransitConstants.TRANSIT_MOTOR_SPEED);
        }
    }

    public void SetTransitSpeed(double speed) {
        if (SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED) {
            transitMotor1.set(-speed);
            transitMotor2.set(speed);
        }
    }

    public void transitoff() {
        if (SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED) {
            transitMotor1.set(0);
            transitMotor2.set(0);
        }
    }

    public double getEncoderPosition1() {
        return SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED ? transitEncoder1.getPosition() : 0;
    }

    public double getEncoderPosition2() {
        return SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED ? transitEncoder2.getPosition() : 0;
    }

    public void resetEncoder() {
        if (SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED) {
            transitEncoder1.setPosition(0.0);
            transitEncoder2.setPosition(0.0);
        }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.TRANSIT_SUBSYSTEM_ENABLED) {
            SmartDashboard.putNumber("Transit Speed", getspeed1());
        }
    }
}