package team.gif.robot.subsystems.drivers.swerve.utilities;

public class SwerveConfiguration {
    public SwerveMap deviceMap;
    public SwerveConstants constants;
    public DriveMotorFactory driveMotorFactory;
    public TurnMotorFactory turnMotorFactory;
    public EncoderFactory encoderFactory;

    public SwerveConfiguration(SwerveMap deviceMap, SwerveConstants constants, DriveMotorFactory driveMotorFactory, TurnMotorFactory turnMotorFactory, EncoderFactory encoderFactory) {
        this.deviceMap = deviceMap;
        this.constants = constants;
        this.driveMotorFactory = driveMotorFactory;
        this.turnMotorFactory = turnMotorFactory;
        this.encoderFactory = encoderFactory;
    }
}
