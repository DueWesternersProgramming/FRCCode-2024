package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotConstants.PortConstants;

public class ShooterSubsystem extends SubsystemBase{

    CANSparkMax shooterMotor1, shooterMotor2;
    RelativeEncoder shooterEncoder1, shooterEncoder2;

    public ShooterSubsystem(){
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED){
            shooterMotor1 = new CANSparkMax(PortConstants.CAN.SHOOTER_MOTOR_PORT_1, MotorType.kBrushless);
            shooterMotor2 = new CANSparkMax(PortConstants.CAN.SHOOTER_MOTOR_PORT_2, MotorType.kBrushless);
            shooterMotor1.setIdleMode(IdleMode.kBrake);
            shooterMotor2.setIdleMode(IdleMode.kBrake);
            shooterEncoder1 = shooterMotor1.getEncoder();
            shooterEncoder2 = shooterMotor2.getEncoder();
            shooterMotor1.setInverted(true);
            // shooterMotor1.burnFlash();
            // shooterMotor2.burnFlash();
            resetEncoder();
        }
    }

    public double getspeed1() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? shooterMotor1.get() : 0;
    }

    public double getspeed2() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? shooterMotor2.get(): 0;
    }

    /**
     * @param mode 0 = speaker, 1 = amp
     */
    public void shooterOn(int mode){
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED){
            if (mode == 0){
                shooterMotor1.setVoltage(ShooterConstants.SHOOTER_MOTOR_SPEAKER_VOLTAGE);
                shooterMotor2.setVoltage(ShooterConstants.SHOOTER_MOTOR_SPEAKER_VOLTAGE);
            }
            else if (mode == 1){
                shooterMotor1.setVoltage(ShooterConstants.SHOOTER_MOTOR_AMP_VOLTAGE);
                shooterMotor2.setVoltage(ShooterConstants.SHOOTER_MOTOR_AMP_VOLTAGE);
            }
        }
    }

    public void setShooterSpeed(double speed){
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED){
            shooterMotor1.set(speed);
            shooterMotor2.set(speed);
        }
    }

    public void shooteroff(){
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED){
            shooterMotor1.setVoltage(0.0);
            shooterMotor2.setVoltage(0.0);
        }
    }

    public void shooterReverse(){
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED){
            shooterMotor1.setVoltage(-ShooterConstants.SHOOTER_MOTOR_SPEAKER_VOLTAGE/1.5);
            shooterMotor2.setVoltage(-ShooterConstants.SHOOTER_MOTOR_SPEAKER_VOLTAGE/1.5);
        }
    }

    public double getEncoderPosition1() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? shooterEncoder1.getPosition() : 0;
    }

    public double getEncoderPosition2() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? shooterEncoder2.getPosition() : 0;
    }

    public void resetEncoder() {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED){
            shooterEncoder1.setPosition(0.0);
            shooterEncoder2.setPosition(0.0);
        }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED){
            SmartDashboard.putNumber("Shooter Speed", getspeed1());
        }
    }
}
