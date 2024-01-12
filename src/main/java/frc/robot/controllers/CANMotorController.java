package frc.robot.controllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class CANMotorController implements MotorController {

    public static enum MotorControllerType {
        TSRX,
        SMAX_BRUSHED,
        SMAX_BRUSHLESS
    }

    private final MotorControllerType type;
    private CANSparkMax motorControllerSMAX;

    public CANMotorController(int CANID, MotorControllerType Type) {
        type = Type;

        switch (type) {
            case SMAX_BRUSHED:
                motorControllerSMAX = new CANSparkMax(CANID, MotorType.kBrushed);
            case SMAX_BRUSHLESS:
                motorControllerSMAX = new CANSparkMax(CANID, MotorType.kBrushless);
        }
    }

    @Override
    public void set(double speed) {
        switch (type) {
            case SMAX_BRUSHED:
                motorControllerSMAX.set(speed);
            case SMAX_BRUSHLESS:
                motorControllerSMAX.set(speed);
        }
    }

    @Override
    public double get() {
        switch (type) {
            case SMAX_BRUSHED:
                return motorControllerSMAX.get();
            case SMAX_BRUSHLESS:
                return motorControllerSMAX.get();
            default:
                return 0.0;
        }
    }

    @Override
    public void disable() {
        switch (type) {
            case SMAX_BRUSHED:
                motorControllerSMAX.disable();
            case SMAX_BRUSHLESS:
                motorControllerSMAX.disable();
        }
    }

    @Override
    public void stopMotor() {
        switch (type) {
            case SMAX_BRUSHED:
                motorControllerSMAX.stopMotor();
            case SMAX_BRUSHLESS:
                motorControllerSMAX.stopMotor();
        }
    }

    @Override
    public void setInverted(boolean inverted) {
        switch (type) {
            case SMAX_BRUSHED:
                motorControllerSMAX.setInverted(inverted);
            case SMAX_BRUSHLESS:
                motorControllerSMAX.setInverted(inverted);
        }
    }

    @Override
    public boolean getInverted() {
        switch (type) {
            case SMAX_BRUSHED:
                return motorControllerSMAX.getInverted();
            case SMAX_BRUSHLESS:
                return motorControllerSMAX.getInverted();
            default:
                return false;
        }
    }

    public void feed() {
        switch (type) {
            case SMAX_BRUSHED:
                motorControllerSMAX.set(get());
            case SMAX_BRUSHLESS:
                motorControllerSMAX.set(get());
        }
    }
}
