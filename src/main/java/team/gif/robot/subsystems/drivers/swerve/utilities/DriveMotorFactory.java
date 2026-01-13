package team.gif.robot.subsystems.drivers.swerve.utilities;

@FunctionalInterface
public interface DriveMotorFactory {
    DriveMotor create(int id, SwerveConstants constants);
}
