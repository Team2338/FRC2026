package team.gif.robot.subsystems.drivers.swerve.utilities;

@FunctionalInterface
public interface EncoderFactory {
    Encoder create(int id);
}
