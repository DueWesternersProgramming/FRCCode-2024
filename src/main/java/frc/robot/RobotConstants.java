package frc.robot;

import java.util.List;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public final class RobotConstants {
        public static final class DrivetrainConstants {
                public static final boolean IS_SWERVE_DEBUGGING_ENABLED = false; // Enables extra network tables swerve
                                                                                 // stats and numbers

                public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 0; // These are zero because we set the
                                                                                  // CANCoders to zero in Pheonix Tuner.
                public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 0;
                public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
                public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = 0;

                public static final double DRIVE_BASE_RADIUS_METERS = 0.52705; // Middle of the robot to one of the
                                                                               // swerve
                                                                               // module wheel centers. Use CAD.

                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double MAX_SPEED_METERS_PER_SECOND = 6.0; // 4.42; //4.8;
                public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

                public static final double DIRECTION_SLEW_RATE = 25;// 1.2; // radians per second
                public static final double MAGNITUDE_SLEW_RATE = 25;// 1.8; // 2.0; //1.8; // percent per second (1 =
                                                                    // 100%)
                public static final double ROTATIONAL_SLEW_RATE = 10;// 2.0; // 20.0; //2.0; // percent per second (1 =
                                                                     // 100%)

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

        public static final class DriverAssistConstants {

                public static final double MAX_VELOCITY = 3.5; // Meters per second
                public static final double MAX_ACCELERATION = 3; // Meters per second squared
                public static final double MAX_ANGULAR_SPEED = 350; // Degrees per second
                public static final double MAX_ANGULAR_ACCELERATION = 500; // Degrees per second squared
                // Create the constraints to use while pathfinding (Speeds and acceleration)
                public static final PathConstraints PATH_FINDING_CONSTRAINTS = new PathConstraints(
                                DriverAssistConstants.MAX_VELOCITY, DriverAssistConstants.MAX_ACCELERATION,
                                Units.degreesToRadians(DriverAssistConstants.MAX_ANGULAR_SPEED),
                                Units.degreesToRadians(DriverAssistConstants.MAX_ANGULAR_ACCELERATION));

                public static final double SKIP_LANE_PATH_DISTANCE = 3; // Meters

        }

        public static final class SwerveModuleConstants {

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
                public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50)
                                / (DRIVING_MOTOR_PINION_TEETH * 15 * 27);
                public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                                * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

                public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS
                                * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
                public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS
                                * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

                public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // ratio between internal relative
                                                                                  // encoder and
                                                                                  // Through Bore (or Thrifty in our
                                                                                  // case)
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

                public static final double TURNING_P = 1.25; // 1.0; // 1.0 might be a bit too much - reduce a bit if
                                                             // needed
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
                        public static final double JOYSTICK_AXIS_DEADZONE = 0.05;
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

                public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(3.5, 0, 0);

                public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(3, 0, 0);

                public static final double FIELD_LENGTH_INCHES = 54 * 12 + 1; // 54ft 1in
                public static final double FIELD_WIDTH_INCHES = 26 * 12 + 7; // 26ft 7in

                public static final double kMaxAngularAcceleration = 4 * Math.PI;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3.00;

                public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
                                DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
                                AutonomousConstants.kMaxAccelerationMetersPerSecondSquared,
                                DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                5 * Math.PI);

                // public static final Boolean FLIP_PATHPLANNER_AUTOS = true;
        }

        public static final class VisionConstants {
                public static final Transform3d[] CAMERA_POSITIONS = {
                                new Transform3d(
                                                // Front Left
                                                new Translation3d(
                                                                Units.inchesToMeters(13.682), // forward+
                                                                Units.inchesToMeters(13.118), // left+
                                                                Units.inchesToMeters(8.351)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(-30), // Note, these are all
                                                                                             // counter clockwise
                                                                                             // positive so to
                                                                                             // face up we
                                                                                             // need -40 (the rest of
                                                                                             // these as well) ;)
                                                                Units.degreesToRadians(20))), // left 20
                                // // Front Right
                                // new Transform3d(
                                // new Translation3d(
                                // Units.inchesToMeters(13.682), // forward+
                                // Units.inchesToMeters(-13.118), // left+
                                // Units.inchesToMeters(8.351)), // up+
                                // new Rotation3d(
                                // Units.degreesToRadians(0),
                                // Units.degreesToRadians(-30),
                                // Units.degreesToRadians(-20))), // right 20
                                // Back Left
                                new Transform3d(
                                                new Translation3d(
                                                                Units.inchesToMeters(-13.774), // forward+
                                                                Units.inchesToMeters(12.645), // left+
                                                                Units.inchesToMeters(8.351)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(-30),
                                                                Units.degreesToRadians(180 - 10))),
                                // Back Right
                                new Transform3d(
                                                new Translation3d(
                                                                Units.inchesToMeters(-13.736), // forward+
                                                                Units.inchesToMeters(-12.817), // left+
                                                                Units.inchesToMeters(8.351)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(-30),
                                                                Units.degreesToRadians(180 + 10)))
                };

                public static final String[] CAMERA_NAMES = { "frontLeftCamera", "backLeftCamera",
                                "backRightCamera" };
        }

        public static final class TeleopConstants {
                public static final double MAX_SPEED_PERCENT = 1; // drivetrain speed percent

                // Driver constants:
                public static final int DRIVE_COMMAND_X_AXIS = 0; // Left X
                public static final int DRIVE_COMMAND_Y_AXIS = 1; // Left Y
                public static final int DRIVE_COMMAND_ROT_AXIS = 4; // Right X

                public static final int ROBOT_RELATIVE_BUTTON = 6;
                public static final int SPEAKER_AIM_BUTTON = 1;
                public static final int RESET_GYRO_BUTTON = 5;
                public static final int X_LOCK_BUTTON = 6;
                public static final int SOURCE_ASSIST_AXIS = 2; // Left trigger
                public static final int SPEAKER_ASSIST_AXIS = 3; // Left trigger

                // Operator constants: TODO:
                public static final int CLIMBER_LEFT_COMMAND_AXIS = 1; // left Y
                public static final int CLIMBER_RIGHT_COMMAND_AXIS = 5; // right Y
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
                public static final boolean CLIMBER_SUBSYSTEM_ENABLED = false;
                public static final boolean INTAKE_SUBSYSTEM_ENABLED = true;
                public static final boolean SHOOTER_SUBSYSTEM_ENABLED = true;
                public static final boolean TRANSIT_SUBSYSTEM_ENABLED = true;
                public static final boolean LIGHT_SUBSYSTEM_ENABLED = false;
                public static final boolean VISION_SUBSYSTEM_ENABLED = true;
        }

        public static final class FieldPointPoses {
                public static Pose2d BLUE_ALLIANCE_SPEAKER = new Pose2d(1.4, 5.55, new Rotation2d(Math.toRadians(0))); // Meters
                public static Pose2d RED_ALLIANCE_SPEAKER = new Pose2d(15.4, 5.55, new Rotation2d(Math.toRadians(180))); // Meters

                public static Pose2d BLUE_ALLIANCE_SOURCE = new Pose2d(15.5, 0.55, new Rotation2d()); // Meters
                public static Pose2d RED_ALLIANCE_SOURCE = new Pose2d(0.85, 0.55, new Rotation2d()); // Meters

                // TODO:ADD PLACEHOLDER DEFAULT LANE
                public static final class BlueAlliance {

                        public static final List<Waypoint> LEFT_LANE_WAYPOINTS = PathPlannerPath
                                        .waypointsFromPoses(
                                                        new Pose2d(8.5, 6.7, Rotation2d.fromDegrees(180)),
                                                        new Pose2d(5.5, 6.6, Rotation2d.fromDegrees(-175)),
                                                        new Pose2d(2.5, 5.7, Rotation2d.fromDegrees(-150)));

                        public static final List<Waypoint> MIDDLE_LANE_WAYPOINTS = PathPlannerPath.waypointsFromPoses(
                                        new Pose2d(7, 4, Rotation2d.fromDegrees(180)),
                                        new Pose2d(5.6, 4, Rotation2d.fromDegrees(180)),
                                        new Pose2d(3.1, 5.75, Rotation2d.fromDegrees(180)));

                        public static final List<Waypoint> RIGHT_LANE_WAYPOINTS = PathPlannerPath.waypointsFromPoses(
                                        new Pose2d(6.5, 1.3, Rotation2d.fromDegrees(160)),
                                        new Pose2d(3.4, 2.4, Rotation2d.fromDegrees(150)),
                                        new Pose2d(1.75, 4, Rotation2d.fromDegrees(130)));
                }

                public static final class RedAlliance {

                        public static final List<Waypoint> LEFT_LANE_WAYPOINTS = PathPlannerPath
                                        .waypointsFromPoses(
                                                        new Pose2d(10, 1.5, Rotation2d.fromDegrees(10)),
                                                        new Pose2d(13.5, 2.4, Rotation2d.fromDegrees(20)),
                                                        new Pose2d(15, 4.25, Rotation2d.fromDegrees(60)));

                        public static final List<Waypoint> MIDDLE_LANE_WAYPOINTS = PathPlannerPath.waypointsFromPoses(
                                        new Pose2d(10, 4, Rotation2d.fromDegrees(10)),
                                        new Pose2d(12, 4.5, Rotation2d.fromDegrees(45)),
                                        new Pose2d(13.5, 6, Rotation2d.fromDegrees(45)));

                        public static final List<Waypoint> RIGHT_LANE_WAYPOINTS = PathPlannerPath.waypointsFromPoses(
                                        new Pose2d(10, 6.75, Rotation2d.fromDegrees(-10)),
                                        new Pose2d(12.4, 6.1, Rotation2d.fromDegrees(-30)),
                                        new Pose2d(14, 5.2, Rotation2d.fromDegrees(-50)));
                }

        }
}