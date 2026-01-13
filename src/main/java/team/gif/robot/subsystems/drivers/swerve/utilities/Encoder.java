package team.gif.robot.subsystems.drivers.swerve.utilities;

/**
 * An interface for encoders used in swerve modules.
 * This is to allow for different types of encoders
 * to be used interchangeably.
 * @implNote The constructor should only accept the CAN ID
 */
public interface Encoder {

    /**
     * configure the encoder, set things like
     * pulling rate and conversion factor.
     */
    void configure();

    /**
     * It is recommended to replace this javadoc
     * with one that specifies the number of ticks per rotation
     * @return double - the current position of the encoder in ticks
     */
    double getTicks();

    /**
     * @return double - current position of the wheel in degrees
     */
    double getDegrees();

    /**
     * @return double - current position of the wheel in radians
     */
    double getRadians();

    /**
     * @return double - the current velocity of the wheel (turning)
     */
    double getVelocity();

    /**
     * Resets the encoder to 0
     */
    void reset();

}
