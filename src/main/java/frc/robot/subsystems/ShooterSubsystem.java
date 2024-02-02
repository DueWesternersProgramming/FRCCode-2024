package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotConstants.PortConstants;

public class ShooterSubsystem extends SubsystemBase{
    CANSparkMax shooterMotor1 = new CANSparkMax(PortConstants.CAN.SHOOTER_MOTOR_PORT_1, MotorType.kBrushless);
    CANSparkMax shooterMotor2 = new CANSparkMax(PortConstants.CAN.SHOOTER_MOTOR_PORT_2, MotorType.kBrushless);
    RelativeEncoder shooterEncoder1 = shooterMotor1.getEncoder();
    RelativeEncoder shooterEncoder2 = shooterMotor2.getEncoder();

    public ShooterSubsystem(){
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        shooterMotor2.setIdleMode(IdleMode.kCoast);
        resetEncoder();
    }

    public double getspeed1() {
        return shooterMotor1.get();
    }

    public double getspeed2() {
        return shooterMotor2.get();
    }

    public void shooterOn(){
        shooterMotor1.set(ShooterConstants.SHOOTER_MOTOR_SPEED);
        shooterMotor2.set(-ShooterConstants.SHOOTER_MOTOR_SPEED);
    }

    public void setShooterSpeed(double speed){
        shooterMotor1.set(speed);
        shooterMotor2.set(-speed);
    }

    public void shooteroff(){
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public double getEncoderPosition1() {
        return shooterEncoder1.getPosition();
    }

    public double getEncoderPosition2() {
        return shooterEncoder2.getPosition();
    }

    public void resetEncoder() {
        shooterEncoder1.setPosition(0.0);
        shooterEncoder2.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", getspeed1());
    }
}
