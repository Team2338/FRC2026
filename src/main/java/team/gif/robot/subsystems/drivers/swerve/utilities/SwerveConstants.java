package team.gif.robot.subsystems.drivers.swerve.utilities;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.subsystems.drivers.swerve.SwerveDrivetrain;

public abstract class SwerveConstants {
    //region Module Configuration
    /**
     * This is the drive gear ratio.
     * It should be found from the swerve module manufacturer.
     * <br><br>Default Value: 1
     */
    public double MODULE_GEAR_RATIO = 1;
    /**
     * This is the turning gear ratio.
     * It should be found from the swerve module manufacturer.
     * <b>This is NOT the gear ratio between the turn motor and the dedicated encoder.<b/>
     * <br><br>Default Value: 1
     */
    public double TURNING_MOTOR_GEAR_RATIO = 1;
    /**
     * The diameter of the wheel in inches.
     * This is converted to meters for calculations.
     * <br><br>Default Value: 4 inches
     */
    public double WHEEL_DIAMETER_INCHES = 4;
    /**
     * The number of encoder counts per revolution. Many newer encoders count in
     * rotations instead of counts. In that instance, the value should be 1.
     * <br><br>Default Value: 1
     */
    public double DRIVE_ENCODER_CPR = 1;
    /**
     * The coefficient of friction of the wheels on the driving surface.
     * This value is passed to PathPlanner, see the PathPlanner documentation
     * for measurement instructions
     * <br><br>Default Value: 1
     */
    public double COEFFICIENT_OF_FRICTION = 1;

    /**
     * This is the maximum speed the motors can propel the robot in a straight line.
     * This needs to be remeasured for every robot. Use Phoenix Tuner or
     * Rev Hardware Client to ensure 100% duty cycle is being applied.
     * <br><br>Default Value: 5 m/s
     */
    public double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;

    /**
     * This is the maximum angular speed the robot can rotate in place.
     * This needs to be remeasured for every robot. Use Phoenix Tuner or
     * Rev Hardware Client to ensure 100% duty cycle is being applied.
     * <br><br>Default Value: 4π radians/s (2 rotations/s)
     */
    public double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;

    /**
     * The sets the maximum acceleration for teleop driving.
     * This is set to ease control of the robot.
     * <br><br>Default Value: 10 m/s²
     */
    public double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 10;

    /**
     * The set maximum angular acceleration for teleop driving.
     * This is set to ease control of the robot.
     * <br><br>Default Value: 5π radians/s²
     */
    public double MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED = 5 * Math.PI;

    /**
     * The inversion state of the FL Drive Motor.
     * <br><br>Default Value: true
     */
    public boolean FL_DRIVE_INVERTED = true;
    /**
     * The inversion state of the FR Drive Motor.
     * <br><br>Default Value: true
     */
    public boolean FR_DRIVE_INVERTED = true;
    /**
     * The inversion state of the RL Drive Motor.
     * <br><br>Default Value: true
     */
    public boolean RL_DRIVE_INVERTED = true;
    /**
     * The inversion state of the RR Drive Motor.
     * <br><br>Default Value: true
     */
    public boolean RR_DRIVE_INVERTED = true;

    /**
     * The inversion state of the FL Turn Motor.
     * <br><br>Default Value: true
     */
    public boolean FL_TURN_INVERTED = true;
    /**
     * The inversion state of the FR Turn Motor.
     * <br><br>Default Value: true
     */
    public boolean FR_TURN_INVERTED = true;
    /**
     * The inversion state of the RL Turn Motor.
     * <br><br>Default Value: true
     */
    public boolean RL_TURN_INVERTED = true;
    /**
     * The inversion state of the RR Turn Motor.
     * <br><br>Default Value: true
     */
    public boolean RR_TURN_INVERTED = true;

    /**
     * The type of motor used in the swerve module.
     * This is passed to PathPlanner.
     * <br><br>No Default Value
     * @see DCMotor
     */
    public DCMotor PATHPLANNER_MOTOR_TYPE;
    /**
     * The current limit for the drive motor.
     * This is passed to PathPlanner.
     * <br><br>No Default Value
     */
    public int PATHPLANNER_CURRENT_LIMIT;

    /**
     * The maximum temperature the drive motor should reach. The unit should be the same as the unit the motor will report its temperature in.
     * <br><br>Default Value: 85
     */
    public double MAX_DRIVE_TEMP =  85;

    /**
     * The diameter of the wheel in meters.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * it will get overridden.
     */
    public double WHEEL_DIAMETER_METERS;

    /**
     * The conversion factor for the drive encoder to meters.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * it will get overridden.
     */
    public double DRIVE_ENCODER_ROT_2_METER;
    /**
     * The conversion factor for the drive encoder to meters per second.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * it will get overridden.
     */
    public double DRIVE_ENCODER_RPM_2_METER_PER_SEC;
    /**
     * The conversion factor for the turning encoder to radians.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * it will get overridden.
     */
    public double TURNING_ENCODER_ROT_TO_RAD;
    /**
     * The conversion factor for the turning encoder to radians per second.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * it will get overridden.
     */
    public double TURNING_ENCODER_RPM_2_RAD_PER_SECOND;
    /**
     * This is the {@link ModuleConfig} object created for PathPlanner.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * it will get overridden.
     */
    public ModuleConfig PATHPLANNER_MODULE_CONFIG;
    //endregion

    //region Drivetrain Configuration
    /**
     * The Front Left absolute encoder offset.
     *
     * <br><br>Default Value: 0
     * <br> Unit: degrees
     */
    public double FRONT_LEFT_OFFSET = 0;
    /**
     * The Front Right absolute encoder offset.
     *
     * <br><br>Default Value: 0
     * <br> Unit: degrees
     */
    public double FRONT_RIGHT_OFFSET = 0;
    /**
     * The Rear Left absolute encoder offset.
     *
     * <br><br>Default Value: 0
     * <br> Unit: degrees
     */
    public double REAR_LEFT_OFFSET = 0;
    /**
     * The Rear Right absolute encoder offset.
     *
     * <br><br>Default Value: 0
     * <br> Unit: degrees
     */
    public double REAR_RIGHT_OFFSET = 0;
    /**
     * The distance between the centers of right and left swerve modules.
     * <b>This values must be defined in {@link #setConfiguration()}</b>
     */
    public double TRACK_LENGTH_INCHES;
    /**
     * The distance between the centers of the front and back swerve modules.
     * <b>This values must be defined in {@link #setConfiguration()}</b>
     */
    public double TRACK_WIDTH_INCHES;
    /**
     * The percentage of the max speed for "coast" mode.
     * It is derived from {@link #PHYSICAL_MAX_SPEED_METERS_PER_SECOND}.
     * This should be between {@code 0} and {@code 1}.
     * <br><br>Default Value: 0.6 (60% of max speed)
     */
    public double COAST_DRIVE_PERC = 0.6;
    /**
     * The percentage of the max speed for "boost" mode.
     * It is derived from {@link #PHYSICAL_MAX_SPEED_METERS_PER_SECOND}.
     * This should be between {@code 0} and {@code 1}.
     * <br><br>Default Value: 1 (100% of max speed)
     */
    public double BOOST_DRIVE_PERC = 1;
    /**
     * The percentage of the max speed for "slow" mode.
     * It is derived from {@link #PHYSICAL_MAX_SPEED_METERS_PER_SECOND}.
     * This should be between {@code 0} and {@code 1}.
     * <br><br>Default Value: 0.3 (30% of max speed)
     */
    public double SLOW_DRIVE_PERC = 0.3;
    /**
     * The mass of the robot in kilograms. This is passed to PathPlanner.
     * <br><br><b>This value must be set in {@link #setConfiguration()}</b>
     */
    public double MASS_KG;
    /**
     * The moment of inertia of the robot in kilogram meters^2. This is passed to PathPlanner.
     * <br><br><b>This value must be set in {@link #setConfiguration()}</b>
     */
    public double MOI_KGM2;

    /**
     * The distance between the center of the left and right swerve modules in meters.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * or it will be overridden.
     */
    public double TRACK_LENGTH_METERS;
    /**
     * The distance between the center of the front and back swerve modules in meters.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * or it will be overridden.
     */
    public double TRACK_WIDTH_METERS;
    /**
     * The kinematics object for the drivetrain.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * or it will be overridden.
     */
    public SwerveDriveKinematics DRIVE_KINEMATICS;
    /**
     * The max speed for "slow" mode in meters per second.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * or it will be overridden.
     */
    public double SLOW_SPEED_METERS_PER_SECOND;
    /**
     * The max speed for "coast" mode in meters per second.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * or it will be overridden.
     */
    public double COAST_SPEED_METERS_PER_SECOND;
    /**
     * The max speed for "boost" mode in meters per second.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * or it will be overridden.
     */
    public double BOOST_SPEED_METERS_PER_SECOND;
    //endregion

    //region PID Configuration
    /**
     * The proportional gain for the turning motors.
     * This is used for all turning motors unless overridden.
     * <br><br><b>This value must be defined in {@link #setConfiguration()}</b>
     */
    public double TURN_P;
    /**
     * The feedforward gain for the turning motors.
     * This is used for all turning motors unless overridden.
     * <br><br><b>This value must be defined in {@link #setConfiguration()}</b>
     */
    public double TURN_FF;
    /**
     * This is the feedforward object used for the front left drive motor.
     * This value should be obtained using SysID.
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html">WPILib Documentation</a>
     * @see SwerveDrivetrain#sysIdDynamic(String, SysIdRoutine.Direction)
     * @see SwerveDrivetrain#sysIdQuasistatic(String, SysIdRoutine.Direction)
     */
    public SimpleMotorFeedforward FL_DRIVE_FF;
    /**
     * This is the feedforward object used for the front right drive motor.
     * This value should be obtained using SysID.
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html">WPILib Documentation</a>
     * @see SwerveDrivetrain#sysIdDynamic(String, SysIdRoutine.Direction)
     * @see SwerveDrivetrain#sysIdQuasistatic(String, SysIdRoutine.Direction)
     */
    public SimpleMotorFeedforward FR_DRIVE_FF;
    /**
     * This is the feedforward object used for the rear left drive motor.
     * This value should be obtained using SysID.
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html">WPILib Documentation</a>
     * @see SwerveDrivetrain#sysIdDynamic(String, SysIdRoutine.Direction)
     * @see SwerveDrivetrain#sysIdQuasistatic(String, SysIdRoutine.Direction)
     */
    public SimpleMotorFeedforward RL_DRIVE_FF;
    /**
     * This is the feedforward object used for the rear right drive motor.
     * This value should be obtained using SysID.
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html">WPILib Documentation</a>
     * @see SwerveDrivetrain#sysIdDynamic(String, SysIdRoutine.Direction)
     * @see SwerveDrivetrain#sysIdQuasistatic(String, SysIdRoutine.Direction)
     */
    public SimpleMotorFeedforward RR_DRIVE_FF;
    /**
     * This is the proportional gain passed to PathPlanner for path following.
     * <br><br>Default Value: 2.5
     */
    public double AUTO_P_FORWARD = 2.5;
    /**
     * This is the proportional gain passed to PathPlanner for path following.
     * <br><br>Default Value: 2.5
     */
    public double AUTO_P_ROTATION = 2.5;

    //NaN means use default (The P and FF values above)
    /**
     * The proportional gain for the front left turning motor. This field will override {@link #TURN_P} for the front left module if set.
     */
    public double FL_P = Double.NaN;
    /**
     * The proportional gain for the front right turning motor. This field will override {@link #TURN_P} for the front right module if set.
     */
    public double FR_P = Double.NaN;
    /**
     * The proportional gain for the rear left turning motor. This field will override {@link #TURN_P} for the rear left module if set.
     */
    public double RL_P = Double.NaN;
    /**
     * The proportional gain for the rear right turning motor. This field will override {@link #TURN_P} for the rear right module if set.
     */
    public double RR_P = Double.NaN;
    /**
     * The feedforward gain for the front left turning motor. This field will override {@link #TURN_FF} for the front left module if set.
     */
    public double FL_FF = Double.NaN;
    /**
     * The feedforward gain for the front right turning motor. This field will override {@link #TURN_FF} for the front right module if set.
     */
    public double FR_FF = Double.NaN;
    /**
     * The feedforward gain for the rear left turning motor. This field will override {@link #TURN_FF} for the rear left module if set.
     */
    public double RL_FF = Double.NaN;
    /**
     * The feedforward gain for the rear right turning motor. This field will override {@link #TURN_FF} for the rear right module if set.
     */
    public double RR_FF = Double.NaN;
    /**
     * PathPlanner's drive controller.
     * This is a derived value that should not be set in {@link #setConfiguration()},
     * or it will get overridden.
     */
    public PPHolonomicDriveController AUTO_DRIVE_CONTROLLER;
    //endregion

    public SwerveConstants() {
        setConfiguration();
        calculate();
    }

    protected abstract void setConfiguration();

    private void calculate() {
        // Compute module derived values
        WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
        DRIVE_ENCODER_ROT_2_METER = (Math.PI * WHEEL_DIAMETER_METERS) / (MODULE_GEAR_RATIO * DRIVE_ENCODER_CPR);
        DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        // Compute drivetrain derived values
        TRACK_LENGTH_METERS = Units.inchesToMeters(TRACK_LENGTH_INCHES);
        TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);
        DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),   // front left
                new Translation2d(TRACK_LENGTH_METERS / 2, -TRACK_WIDTH_METERS / 2),  // front right
                new Translation2d(-TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),  // back left
                new Translation2d(-TRACK_LENGTH_METERS / 2, -TRACK_WIDTH_METERS / 2)  // back right
        );
        SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_PERC * PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        COAST_SPEED_METERS_PER_SECOND = COAST_DRIVE_PERC * PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_PERC * PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

        // Compute PID derived values
        //Check to see if override exists, otherwise use default
        FL_P = Double.isNaN(FL_P) ? TURN_P : FL_P;
        FR_P = Double.isNaN(FR_P) ? TURN_P : FR_P;
        RL_P = Double.isNaN(RL_P) ? TURN_P : RL_P;
        RR_P = Double.isNaN(RR_P) ? TURN_P : RR_P;
        FL_FF = Double.isNaN(FL_FF) ? TURN_FF : FL_FF;
        FR_FF = Double.isNaN(FR_FF) ? TURN_FF : FR_FF;
        RL_FF = Double.isNaN(RL_FF) ? TURN_FF : RL_FF;
        RR_FF = Double.isNaN(RR_FF) ? TURN_FF : RR_FF;
        AUTO_DRIVE_CONTROLLER = new PPHolonomicDriveController(
                new PIDConstants(AUTO_P_FORWARD, 0.0, 0.0),
                new PIDConstants(AUTO_P_ROTATION, 0.0, 0.0)
        );

        PATHPLANNER_MODULE_CONFIG = new ModuleConfig(
                WHEEL_DIAMETER_METERS,
                PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
                COEFFICIENT_OF_FRICTION,
                PATHPLANNER_MOTOR_TYPE,
                PATHPLANNER_CURRENT_LIMIT,
                1
        );
    }
}