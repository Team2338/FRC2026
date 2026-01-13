package team.gif.robot.subsystems.drivers.swerve.utilities;

@FunctionalInterface
public interface TurnMotorFactory {
    TurnMotor create(int id);
}
