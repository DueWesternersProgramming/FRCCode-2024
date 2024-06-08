package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotContainer.UserPolicy;
import frc.robot.RobotConstants.PortConstants;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax shooterMotor1, shooterMotor2;
    RelativeEncoder shooterEncoder1, shooterEncoder2;
    // DriveSubsystem driveSubsystem;

    public ShooterSubsystem() {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            shooterMotor1 = new CANSparkMax(PortConstants.CAN.SHOOTER_MOTOR_PORT_1, MotorType.kBrushless);
            shooterMotor2 = new CANSparkMax(PortConstants.CAN.SHOOTER_MOTOR_PORT_2, MotorType.kBrushless);
            shooterMotor1.setIdleMode(IdleMode.kBrake);
            shooterMotor2.setIdleMode(IdleMode.kBrake);
            shooterEncoder1 = shooterMotor1.getEncoder();
            shooterEncoder2 = shooterMotor2.getEncoder();
            shooterMotor1.setInverted(true);
            // driveSubsystem = new DriveSubsystem();
            // shooterMotor1.burnFlash();
            // shooterMotor2.burnFlash();
            resetEncoder();
        }
    }

    public double getspeed1() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? shooterMotor1.get() : 0;
    }

    public double getspeed2() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? shooterMotor2.get() : 0;
    }

    /**
     * @param mode 0 = speaker, 1 = amp
     */
    public void shooterOn(int mode) {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            if (mode == 0) {
                shooterMotor1.setVoltage(ShooterConstants.SHOOTER_MOTOR_SPEAKER_VOLTAGE);
                shooterMotor2.setVoltage(ShooterConstants.SHOOTER_MOTOR_SPEAKER_VOLTAGE - 8.0);
            } else if (mode == 1) {
                shooterMotor1.setVoltage(ShooterConstants.SHOOTER_MOTOR_AMP_VOLTAGE);
                shooterMotor2.setVoltage(ShooterConstants.SHOOTER_MOTOR_AMP_VOLTAGE);
            }
        }
    }

    public void setShooterSpeed(double speed) {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            shooterMotor1.set(speed);
            shooterMotor2.set(speed);
        }
    }

    public void shooterOff() {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            shooterMotor1.setVoltage(0.0);
            shooterMotor2.setVoltage(0.0);
        }
    }

    public void shooterReverse() {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            shooterMotor1.setVoltage(-ShooterConstants.SHOOTER_MOTOR_SPEAKER_VOLTAGE / 1.3);
            shooterMotor2.setVoltage(-ShooterConstants.SHOOTER_MOTOR_SPEAKER_VOLTAGE / 1.3);
        }
    }

    public double getEncoderPosition1() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? shooterEncoder1.getPosition() : 0;
    }

    public double getEncoderPosition2() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? shooterEncoder2.getPosition() : 0;
    }

    public void resetEncoder() {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            shooterEncoder1.setPosition(0.0);
            shooterEncoder2.setPosition(0.0);
        }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.SHOOTER_SUBSYSTEM_ENABLED) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                if ((DriveSubsystem.getPose().get().getX() < 2) && (UserPolicy.intakeRunning)) {
                    shooterOff();
                } else if ((DriveSubsystem.getPose().get().getX() < 2) && (UserPolicy.intakeRunning == false)
                        && (IntakeSubsystem.getVelocity() < 5)) {
                    shooterOn(0);

                }

                if (DriveSubsystem.getPose().get().getX() > 2) {
                    shooterOff();
                }
            }
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                if (DriveSubsystem.getPose().get().getX() < 14.5) {
                    shooterOff();
                }
                if (DriveSubsystem.getPose().get().getX() > 14.5) {
                    shooterOn(0);
                }

            }

        }

    }
}
