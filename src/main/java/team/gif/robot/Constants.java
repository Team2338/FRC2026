// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
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

    //These constants should be referenced via Robot.swerveDrive.getConstants();
    public static final class Mk5Constants extends SwerveConstants {
        @Override
        protected void setConfiguration() {
            MODULE_GEAR_RATIO = 6.03;
            TURNING_MOTOR_GEAR_RATIO = 1;
            COEFFICIENT_OF_FRICTION = 1; //TODO: find
            WHEEL_DIAMETER_INCHES = 4.0;

            FRONT_LEFT_OFFSET = -163.388671875;
            FRONT_RIGHT_OFFSET = -15.64453125;
            REAR_LEFT_OFFSET = 18.193359375;
            REAR_RIGHT_OFFSET = -140.537109375;

            FR_DRIVE_INVERTED = false;
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

            AUTO_P_FORWARD = 2.5;
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
}