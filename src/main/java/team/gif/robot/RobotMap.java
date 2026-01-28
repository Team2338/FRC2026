package team.gif.robot;

import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveMap;

public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

    public static final class Mk4Map extends SwerveMap {
        @Override
        protected void setIDs() {
            // SwerveDrivetrain IDs
            /*                As viewed from center of bot
             *              Drive Motor        Turn Motor           CAN
             *            Left(1) Right(2)  Left(3) Right(4)  Left(5) Right(6)
             *   Front(1)   11     12          13      14       15      16
             *   Rear(2)    21     22          23      24       25      26
             *
             */
            FRONT_LEFT_DRIVE_MOTOR_ID = 11;
            FRONT_RIGHT_DRIVE_MOTOR_ID = 12;
            REAR_LEFT_DRIVE_MOTOR_ID = 21;
            REAR_RIGHT_DRIVE_MOTOR_ID = 22;

            FRONT_LEFT_TURNING_MOTOR_ID = 13;
            FRONT_RIGHT_TURNING_MOTOR_ID = 14;
            REAR_LEFT_TURNING_MOTOR_ID = 23;
            REAR_RIGHT_TURNING_MOTOR_ID = 24;

            FRONT_LEFT_ENCODER_ID = 15;
            FRONT_RIGHT_ENCODER_ID = 16;
            REAR_LEFT_ENCODER_ID = 25;
            REAR_RIGHT_ENCODER_ID = 26;
        }
    }

    public static final class Mk3Map extends SwerveMap {
        @Override
        protected void setIDs() {
            FRONT_LEFT_DRIVE_MOTOR_ID = 14;
            FRONT_RIGHT_DRIVE_MOTOR_ID = 20;
            REAR_LEFT_DRIVE_MOTOR_ID = 34;
            REAR_RIGHT_DRIVE_MOTOR_ID = 1;

            FRONT_LEFT_TURNING_MOTOR_ID = 7;
            FRONT_RIGHT_TURNING_MOTOR_ID = 31;
            REAR_LEFT_TURNING_MOTOR_ID = 9;
            REAR_RIGHT_TURNING_MOTOR_ID = 32;
        }
    }

    public static final int PIGEON_ID = 9;

    public static final int SPARK_ID = 60;
    public static final int SPARK_2_ID = 61;
    public static final int TALON_ID = 1;
}
