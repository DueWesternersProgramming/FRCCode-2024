package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.RobotConstants.PortConstants;


public class IntakeSubsystem extends SubsystemBase{
    
    CANSparkMax intakeMotor = new CANSparkMax(PortConstants.CAN.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    public IntakeSubsystem(){
        intakeMotor.setIdleMode(IdleMode.kBrake);
        resetEncoder();
    }

    public double getSpeed() {
        return intakeMotor.get();
    }

    public void intakeOn(){
        intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void intakeReverse(){
        intakeMotor.set(IntakeConstants.INTAKE_REVERSE_MOTOR_SPEED);
    }

    public void intakeOff(){
        intakeMotor.set(0);
    }

    public double getEncoderPosition() {
        return intakeEncoder.getPosition();
    }

    public void resetEncoder() {
        intakeEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", getSpeed());
    }
}