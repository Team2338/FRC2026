// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DEBOUNCE_DEFAULT = 0.020;

    public static final class Field {
        public static final Translation2d HUB_BLUE_TRANSLATION = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
        public static final Translation2d HUB_RED_TRANSLATION = new Translation2d(Units.inchesToMeters((651.22 - 182.11)), Units.inchesToMeters(158.84));
    }

    public static final class Vision {
        public static final Transform3d LEFT_CAMERA_POSITION = new Transform3d(-Units.inchesToMeters(7.925), Units.inchesToMeters(11.6875), Units.inchesToMeters(21.75), new Rotation3d(0, -Units.degreesToRadians(18), 0));
        public static final Transform3d RIGHT_CAMERA_POSITION = new Transform3d(-Units.inchesToMeters(7.925), -Units.inchesToMeters(11.6875), Units.inchesToMeters(21.75), new Rotation3d(0, -Units.degreesToRadians(18), 0));
        public static final Transform3d SIDE_CAMERA_POSITION = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    }

    //These constants should be referenced via Robot.swerveDrive.getConstants();
    public static final class Mk5Constants extends SwerveConstants {
        public static final double HUB_ALIGN_P = 3; //Used in HubAutoAlign to point toward the hub

        @Override
        protected void setConfiguration() {
            MODULE_GEAR_RATIO = 6.03;
            TURNING_MOTOR_GEAR_RATIO = 1;
            COEFFICIENT_OF_FRICTION = 1; //TODO: find
            WHEEL_DIAMETER_INCHES = 4.0;

            FRONT_LEFT_OFFSET = 108.017578125;
            FRONT_RIGHT_OFFSET = -73.212890625;
            REAR_LEFT_OFFSET = 129.111328125;
            REAR_RIGHT_OFFSET = -105.029296875;

            FL_DRIVE_INVERTED = false;
            FR_DRIVE_INVERTED = false;
            RL_DRIVE_INVERTED = false;
            RR_DRIVE_INVERTED = false;

            TRACK_LENGTH_INCHES = 22.5;
            TRACK_WIDTH_INCHES = 22;

            MASS_KG = 16; //TODO: Measure
            MOI_KGM2 = 0; //TODO: Measure

            TURN_P = 0.55; //TODO: Tune
            TURN_FF = 0.01; //TODO: Tune

            FL_DRIVE_FF = new SimpleMotorFeedforward(0.16095, 2.3837, 0.077757); //TODO: Tune
            FR_DRIVE_FF = new SimpleMotorFeedforward(0.1645, 2.3928, 0.074191); //TODO: Tune
            RL_DRIVE_FF = new SimpleMotorFeedforward(0.10265, 2.3955, 0.22997); //TODO: Tune
            RR_DRIVE_FF = new SimpleMotorFeedforward(0.13952, 2.4217, 0.137); //TODO: Tune

            AUTO_P_FORWARD = 2.5; //TODO: Tune
            AUTO_P_ROTATION = 2.5; //TODO: Tune

            PATHPLANNER_MOTOR_TYPE = DCMotor.getKrakenX60(1);
            PATHPLANNER_CURRENT_LIMIT = 50;



        }
    }

    //These constants should be referenced via Robot.swerveDrive.getConstants();
    public static final class Mk4Constants extends SwerveConstants {
        @Override
        protected void setConfiguration() {
            MODULE_GEAR_RATIO = 6.75;
            TURNING_MOTOR_GEAR_RATIO = 12.8;
            WHEEL_DIAMETER_INCHES = 4.0;

            FRONT_LEFT_OFFSET = 79.435125;
            FRONT_RIGHT_OFFSET = -20.09625;
            REAR_LEFT_OFFSET = -137.8125;
            REAR_RIGHT_OFFSET = 155.126953125;

            TRACK_LENGTH_INCHES = 24.899;
            TRACK_WIDTH_INCHES = 21.399;

            MASS_KG = 68;
            MOI_KGM2 = 6.883;

            TURN_P = 0.55;
            TURN_FF = 0.01;

            FL_DRIVE_FF = new SimpleMotorFeedforward(0.16095, 2.3837, 0.077757);
            FR_DRIVE_FF = new SimpleMotorFeedforward(0.1645, 2.3928, 0.074191);
            RL_DRIVE_FF = new SimpleMotorFeedforward(0.10265, 2.3955, 0.22997);
            RR_DRIVE_FF = new SimpleMotorFeedforward(0.13952, 2.4217, 0.137);

            AUTO_P_FORWARD = 1.75;
            AUTO_P_ROTATION = 2.5;

            PATHPLANNER_MOTOR_TYPE = DCMotor.getKrakenX60(1);
            PATHPLANNER_CURRENT_LIMIT = 50;
        }
    }

    public static final class Mk3Constants extends SwerveConstants {
        @Override
        protected void setConfiguration() {
            FRONT_LEFT_OFFSET = 42.8902;
            REAR_LEFT_OFFSET = 358.9453;
            FRONT_RIGHT_OFFSET =  255.4648;
            REAR_RIGHT_OFFSET = 199.0722;

            TURN_P = 0.4;
            TURN_FF = 0.01;

            FL_DRIVE_FF= new SimpleMotorFeedforward(0.16714, 2.7681, 0.41146);
            FR_DRIVE_FF= new SimpleMotorFeedforward(0.10365, 2.7078, 0.49142);
            RL_DRIVE_FF= new SimpleMotorFeedforward(0.10551, 2.8234, 0.48642);
            RR_DRIVE_FF = new SimpleMotorFeedforward(0.073007, 2.75, 0.40028);

            MODULE_GEAR_RATIO = 6.68;
            TURNING_MOTOR_GEAR_RATIO = 12.8;
            WHEEL_DIAMETER_INCHES = 3.78;
            DRIVE_ENCODER_CPR = 42; //Neo Motor

            TRACK_LENGTH_INCHES = 22.5;
            TRACK_WIDTH_INCHES = 23;

            PATHPLANNER_MOTOR_TYPE = DCMotor.getNEO(1);
            PATHPLANNER_CURRENT_LIMIT = 40;
        }
    }

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }

    public static final class Collector {
        public static final double COLLECTOR_SLOW_PERCENT = 0.5;
        public static final double COLLECTOR_FAST_PERCENT = 0.9;

        public static final double PIVOT_PERCENT_MULTIPLIER = 0.70;
        public static final double PIVOT_SOFT_LIMIT_UP_ENCODER_POS = 0.05;
        public static final double PIVOT_LIMIT_AGITATE_POS         = 0.30;
        public static final double PIVOT_LIMIT_AUTO_AGITATE_START_POS = 0.6;
        public static final double PIVOT_DEPLOYED_ENCODER_POS      = 0.6208;

        public static final double PIVOT_POSITION_TOLERANCE = 0.5; //Change value

        public static final double AGITATOR_MOTOR_AUTON_PERCENT = 0.3; //Separate in case we want autos to be different
        public static final double AGITATOR_MOTOR_PERCENT = 0.3;
    }

    public static final class Shooter {
        public static final double SHOOTER_AUTO_SECONDS = 2.5; //Change value
        public static final double SHOOTER_AUTON_FAR_RPM = 3450; //Tune
        public static final double SHOOTER_AUTON_NEAR_RPM = 3000;
    }

    public static final class Indexer {
        public static final double INDEXER_REVERSE_TELEOP_SECONDS = 0.25;
        public static final double INDEXER_REVERSE_PERCENT = -0.25;
        public static final double INDEXER_REVERSE_AUTO_SECONDS = 0.25;

        public static final double BOTTOM_INDEXER_MOTOR_PERCENT = 0.7; //Should change
        public static final double TOP_INDEXER_MOTOR_PERCENT = 0.7; //Should change
    }

    public static final class MotorTemps {
        public static final double COLLECTOR_MOTOR_TEMP_WARNING_CELSIUS = 75.0;
        public static final double PIVOT_MOTOR_TEMP_WARNING_CELSIUS = 75.0;
        public static final double AGITATOR_MOTOR_TEMP_WARNING_CELSIUS = 75.0;
        public static final double BOTTOM_INDEXER_MOTOR_WARNING_CELSIUS = 75.0;
        public static final double TOP_INDEXER_MOTOR_TEMP_WARNING_CELSIUS = 75.0;
        public static final double SHOOTER_MOTOR_TEMP_WARNING_CELSIUS = 75.0;
    }

    public static final class MatchTimes {
        public static final double END_OF_TRANSITION_PERIOD = 130.0;
        public static final double END_OF_FIRST_SHIFT = 105.0;
        public static final double END_OF_SECOND_SHIFT = 80.0;
        public static final double END_OF_THIRD_SHIFT = 55.0;
        public static final double END_OF_FOURTH_SHIFT = 30.0;
        public static final double END_OF_MATCH = 0.0;
    }
}