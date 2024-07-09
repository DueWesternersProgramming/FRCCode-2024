package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class RobotConstants {
    public static final class DrivetrainConstants {
        public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 0;// 5.30603;
        public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 0;// 3.31033;
        public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = 0;// 0.59211;
        public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = 0;// 5.67266;

        public static final double DRIVE_BASE_RADIUS_METERS = 0.52705;

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double MAX_SPEED_METERS_PER_SECOND = 6.0; // 4.42; //4.8;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

        public static final double DIRECTION_SLEW_RATE = 25;// 1.2; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 25;// 1.8; // 2.0; //1.8; // percent per second (1 = 100%)
        public static final double ROTATIONAL_SLEW_RATE = 10;// 2.0; // 20.0; //2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(25);

        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE_METERS = Units.inchesToMeters(25);

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

        public static final int GYRO_ORIENTATION = -1;

        public static final boolean FIELD_RELATIVE = true;
    }

    public static final class PathFindingOnFlyConstants {
        public static final double MAX_VELOCITY = 1; // Meters per second
        public static final double MAX_ACCELERATION = 1; // Meters per second squared
        public static final double MAX_ANGULAR_SPEED = 300.0; // Degrees per second
        public static final double MAX_ANGULAR_ACCELERATION = 150.0; // Degrees per second squared
    }

    public static final class SwerveModuleConstants {

        public static final double TRANSLATION_P = 1.0;
        public static final double ROT_MOTION_P = 0.0;

        public static final double TRANSLATION_I = 0.0;
        public static final double ROT_MOTION_I = 0.0;

        public static final double TRANSLATION_D = 0.0;
        public static final double ROT_MOTION_D = 0.0;

        public static final double FREE_SPEED_RPM = 5676;

        public static final int DRIVING_MOTOR_PINION_TEETH = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50) / (DRIVING_MOTOR_PINION_TEETH * 15 * 27);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS
                * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS
                * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

        public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // ratio between internal relative encoder and
                                                                          // Through Bore (or Thrifty in our case)
                                                                          // absolute encoder - 150.0 / 7.0

        public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI)
                / TURNING_MOTOR_REDUCTION; // radians, per rotation
        public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI)
                / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI); // radians

        public static final double DRIVING_P = 0.07;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
        public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

        public static final double TURNING_P = 1.25; // 1.0; // 1.0 might be a bit too much - reduce a bit if needed
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
        public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

        public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40; // 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps
    }

    public static interface PortConstants {

        public static class CAN {
            public static final int FRONT_LEFT_DRIVING = 5;
            public static final int REAR_LEFT_DRIVING = 7;
            public static final int FRONT_RIGHT_DRIVING = 6;
            public static final int REAR_RIGHT_DRIVING = 8;

            public static final int FRONT_LEFT_TURNING = 9;
            public static final int REAR_LEFT_TURNING = 11;
            public static final int FRONT_RIGHT_TURNING = 10;
            public static final int REAR_RIGHT_TURNING = 12;

            public static final int FRONT_LEFT_STEERING = 1;
            public static final int FRONT_RIGHT_STEERING = 2;
            public static final int REAR_LEFT_STEERING = 3;
            public static final int REAR_RIGHT_STEERING = 4;

            public static final int INTAKE_MOTOR_PORT = 20;

            public static final int LIGHT_PORT = 25;

            public static final int SHOOTER_MOTOR_PORT_1 = 21;
            public static final int SHOOTER_MOTOR_PORT_2 = 22;

            public static final int TRANSIT_MOTOR_PORT_1 = 23;
            public static final int TRANSIT_MOTOR_PORT_2 = 24;

            public static final int LEFT_CLIMBER_PORT = 26;
            public static final int RIGHT_CLIMBER_PORT = 27;
        }

        public static class Controller {
            public static final double JOYSTICK_AXIS_THRESHOLD = 0.2;
            public static final int DRIVE_JOYSTICK = 0;
            public static final int PANEL = 1;
            public static final int OPERATOR_JOYSTICK = 1;
        }
    }

    public static class OffsetConstants {
        public static final double FRONT_OFFSET_HEAVE_M = 0.0;
        public static final double FRONT_OFFSET_SWAY_M = 0.25;
        public static final double FRONT_OFFSET_SURGE_M = 0.0;
        public static final double FRONT_OFFSET_YAW_DEGREES = 45.0;
        public static final double FRONT_OFFSET_PITCH_DEGREES = 0.0;
        public static final double FRONT_OFFSET_ROLL_DEGREES = 0.0;
    }

    public static final class AutonomousConstants {
        public static final double X_CONTROLLER_P = 3.5;
        public static final double Y_CONTROLLER_P = 3.5;
        public static final double THETA_CONTROLLER_P = 3;

        public static final double X_CONTROLLER_I = 0;
        public static final double Y_CONTROLLER_I = 0;
        public static final double THETA_CONTROLLER_I = 0;

        public static final double X_CONTROLLER_D = 0;
        public static final double Y_CONTROLLER_D = 0;
        public static final double THETA_CONTROLLER_D = 0;

        public static final double FIELD_LENGTH_INCHES = 54 * 12 + 1; // 54ft 1in
        public static final double FIELD_WIDTH_INCHES = 26 * 12 + 7; // 26ft 7in
    }

    public static final class VisionConstants {
        public static final double AUTO_ALIGN_P = 0.005;
        public static final double AUTO_ALIGN_I = 0;
        public static final double AUTO_ALIGN_D = 55;
    }

    public static final class TeleopConstants {
        public static final double MAX_SPEED_PERCENT = 0.75;
        public static final int DRIVE_COMMAND_X_AXIS = 0;
        public static final int DRIVE_COMMAND_Y_AXIS = 1;
        
        public static final int DRIVE_COMMAND_ROT_AXIS = 4;
        public static final int SHOOTER_COMMAND_AXIS = 3;
        public static final int CLIMBER_LEFT_COMMAND_AXIS = 1;
        public static final int CLIMBER_RIGHT_COMMAND_AXIS = 5;

        // Driver constants:
        public static final int ROBOT_RELATIVE_BUTTON = 6;
        public static final int SPEAKER_AIM_BUTTON = 1;
        public static final int RESET_GYRO_BUTTON = 5;
        public static final int X_LOCK_BUTTON = 6;

        // Operator constants: TODO:
    }

    public static final class IntakeConstants {
        public static final double INTAKE_MOTOR_SPEED = 0.95;
        public static final double INTAKE_REVERSE_MOTOR_SPEED = -0.40;
        public static final double INTAKE_MOTOR_SHOOT_SPEED = 1;

    }

    public static final class ShooterConstants {
        public static final double SHOOTER_MOTOR_SPEAKER_VOLTAGE = 12.5;
        public static final double SHOOTER_MOTOR_AMP_VOLTAGE = 2.25;
    }

    public static final class TransitConstants {
        public static final double TRANSIT_MOTOR_SPEED = .8;
        public static final double REVERSE_TRANSIT_MOTOR_SPEED = -0.20;
    }

    public static final class LightConstants {
        public static final int LIGHT_COUNT = 8;
        public static final double LIGHT_BRIGHTNESS = 0.25;
    }

    public static final class ClimberConstants {
        public static final double CLIMBER_MOTOR_SPEED = 0.2;
        public static final double CLIMBER_DOWN_POSITION = 0;
        public static final double CLIMBER_UP_POSITION = 100;
    }

    public static final class SubsystemEnabledConstants {
        public static final boolean DRIVE_SUBSYSTEM_ENABLED = true;
        public static final boolean CLIMBER_SUBSYSTEM_ENABLED = true;
        public static final boolean INTAKE_SUBSYSTEM_ENABLED = true;
        public static final boolean SHOOTER_SUBSYSTEM_ENABLED = true;
        public static final boolean TRANSIT_SUBSYSTEM_ENABLED = true;
        public static final boolean LIGHT_SUBSYSTEM_ENABLED = true;
        public static final boolean VISION_SUBSYSTEM_ENABLED = true;
    }
}