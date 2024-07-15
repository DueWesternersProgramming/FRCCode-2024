package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.IntakeConstants;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotContainer.UserPolicy;

public class IntakeSubsystem extends SubsystemBase {

    static CANSparkMax intakeMotor;
    static RelativeEncoder intakeEncoder;
    NetworkTableInstance networkTableInstance;

    public IntakeSubsystem() {
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            intakeMotor = new CANSparkMax(PortConstants.CAN.INTAKE_MOTOR_PORT, MotorType.kBrushless);
            intakeMotor.setIdleMode(IdleMode.kCoast);
            intakeEncoder = intakeMotor.getEncoder();
            resetIntakeEncoder();
            networkTableInstance = NetworkTableInstance.getDefault();
        }
    }

    public static double getIntakeSpeed() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? intakeMotor.get() : 0;
    }

    public void intakeOn() {
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            UserPolicy.intakeRunning = true;
            intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED);
        }
    }

    public void intakeShoot() {
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SHOOT_SPEED);
        }
    }

    public void intakeReverse() {
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            intakeMotor.set(IntakeConstants.INTAKE_REVERSE_MOTOR_SPEED);
        }
    }

    public void intakeOff() {
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            intakeMotor.set(0);
            UserPolicy.intakeRunning = false;
        }
    }

    public double getIntakeEncoderPosition() {
        return SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED ? intakeEncoder.getPosition() : 0;
    }

    public void resetIntakeEncoder() {
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            intakeEncoder.setPosition(0.0);
        }
    }

    public boolean seesNote() {
        try {
            if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
                if (SmartDashboard.getNumber("rawcolorR", 0) > 85) {
                    return true;
                }
            }
        } catch (NumberFormatException e) {

        }
        return false;
    }

    public static double getVelocity() {
        return intakeEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.INTAKE_SUBSYSTEM_ENABLED) {
            SmartDashboard.putNumber("Intake Speed", getIntakeSpeed());
        }
    }

    public Command reverseIntakeCommand() {
        return new StartEndCommand(() -> {
            intakeReverse();
        }, () -> {
        });
    }

    public Command shootIntakeCommand() {
        // This command will feed full speed forward to the transit then to the shooter.
        return new StartEndCommand(() -> {
            intakeShoot();
        }, () -> {
        });
    }

    public Command startIntakeCommand() {
        return new StartEndCommand(() -> {
            intakeOn();
        }, () -> {
        });
    }

    public Command stopIntakeCommand() {
        return new StartEndCommand(() -> {
            intakeOff();
        }, () -> {
        });
    }
}