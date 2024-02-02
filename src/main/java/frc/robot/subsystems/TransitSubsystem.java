package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.TransitConstants;

public class TransitSubsystem extends SubsystemBase{
    CANSparkMax transitMotor1 = new CANSparkMax(TransitConstants.TRANSIT_MOTOR_PORT_1, MotorType.kBrushless);
    CANSparkMax transitMotor2 = new CANSparkMax(TransitConstants.TRANSIT_MOTOR_PORT_2, MotorType.kBrushless);
    RelativeEncoder transitEncoder1 = transitMotor1.getEncoder();
    RelativeEncoder transitEncoder2 = transitMotor2.getEncoder();

    public TransitSubsystem(){
        transitMotor1.setIdleMode(IdleMode.kCoast);
        transitMotor2.setIdleMode(IdleMode.kCoast);
        resetEncoder();
    }

    public double getspeed1() {
        return transitMotor1.get();
    }

    public double getspeed2() {
        return transitMotor2.get();
    }

    public void transitOn(){
        transitMotor1.set(TransitConstants.TRANSIT_MOTOR_SPEED);
        transitMotor2.set(-TransitConstants.TRANSIT_MOTOR_SPEED);
    }

    public void SetTransitSpeed(double speed){
        transitMotor1.set(speed);
        transitMotor2.set(-speed);
    }

    public void transitoff(){
        transitMotor1.set(0);
        transitMotor2.set(0);
    }

    public double getEncoderPosition1() {
        return transitEncoder1.getPosition();
    }

    public double getEncoderPosition2() {
        return transitEncoder2.getPosition();
    }

    public void resetEncoder() {
        transitEncoder1.setPosition(0.0);
        transitEncoder2.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Transit Speed", getspeed1());
    }
}
