package team.gif.robot.subsystems.drivers.swerve.utilities;

public abstract class SwerveMap {
    public int FRONT_LEFT_DRIVE_MOTOR_ID;
    public int FRONT_RIGHT_DRIVE_MOTOR_ID;
    public int REAR_LEFT_DRIVE_MOTOR_ID;
    public int REAR_RIGHT_DRIVE_MOTOR_ID;

    public int FRONT_LEFT_TURNING_MOTOR_ID;
    public int FRONT_RIGHT_TURNING_MOTOR_ID;
    public int REAR_LEFT_TURNING_MOTOR_ID;
    public int REAR_RIGHT_TURNING_MOTOR_ID;

    //These get default values because not all swerve modules have CAN encoders
    public int FRONT_LEFT_ENCODER_ID = 0;
    public int FRONT_RIGHT_ENCODER_ID = 0;
    public int REAR_LEFT_ENCODER_ID = 0;
    public int REAR_RIGHT_ENCODER_ID = 0;

    public SwerveMap() {
        setIDs();
    }

    protected abstract void setIDs();
}