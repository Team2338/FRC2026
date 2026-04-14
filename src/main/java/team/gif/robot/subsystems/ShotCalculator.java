package team.gif.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShotCalculator {
    private final static InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

    static {
//        distanceMap.put(Units.feetToMeters(15.0 + 0.75), 3750.0);
//        distanceMap.put(Units.feetToMeters(13.42 + 0.75), 3650.0);
//        distanceMap.put(Units.feetToMeters(11.42 + 0.75), 3500.0);
//        distanceMap.put(Units.feetToMeters(8.42 + 0.75), 3100.0);
//        distanceMap.put(Units.feetToMeters(7.42 + 0.75), 3050.0);
//        distanceMap.put(Units.feetToMeters(6.42 + 0.75), 3000.0);
//        distanceMap.put(Units.feetToMeters(5.42 + 0.75), 2750.0);
//        distanceMap.put(Units.feetToMeters(4.42 + 0.75), 2650.0);
//        distanceMap.put(Units.feetToMeters(3.42 + 0.75), 2550.0);

        double measurementOffset = 3.35;

//        distanceMap.put(Units.feetToMeters(6.83 + 3.35), 3500.0);
//        distanceMap.put(Units.feetToMeters(4 + 3.35), 3050.0);
//        distanceMap.put(Units.feetToMeters(5.583 + 3.35), 3250.0);
//        distanceMap.put(Units.feetToMeters(12.96 + 3.35), 4300.0);

//        distanceMap.put(Units.feetToMeters(2.42 + measurementOffset), 2800.0);
//        distanceMap.put(Units.feetToMeters(3.94 + measurementOffset), 3000.0);
//        distanceMap.put(Units.feetToMeters(4.98 + measurementOffset), 3150.0);
//        distanceMap.put(Units.feetToMeters(5.79 + measurementOffset), 3250.0);
//        distanceMap.put(Units.feetToMeters(6.76 + measurementOffset), 3350.0);
//        distanceMap.put(Units.feetToMeters(8.82 + measurementOffset), 3550.0);
//        distanceMap.put(Units.feetToMeters(7.5 + measurementOffset), 3450.0);
//        distanceMap.put(Units.feetToMeters(9.92 + measurementOffset), 3850.0);
//        distanceMap.put(Units.feetToMeters(11.67 + measurementOffset), 4150.0);
//        distanceMap.put(Units.feetToMeters(13.92 + measurementOffset), 4300.0);

        distanceMap.put(Units.feetToMeters(2 + measurementOffset), 2650.0);
        distanceMap.put(Units.feetToMeters(3 + measurementOffset), 2700.0);
        distanceMap.put(Units.feetToMeters(3.5 + measurementOffset), 2725.0);
        distanceMap.put(Units.feetToMeters(4 + measurementOffset), 2775.0);
        distanceMap.put(Units.feetToMeters(4.5 + measurementOffset), 2825.0);
        distanceMap.put(Units.feetToMeters(5 + measurementOffset), 2925.0);
        distanceMap.put(Units.feetToMeters(5.5 + measurementOffset), 2975.0);
        distanceMap.put(Units.feetToMeters(6 + measurementOffset), 3050.0);
        distanceMap.put(Units.feetToMeters(6.5 + measurementOffset), 3125.0);
        distanceMap.put(Units.feetToMeters(7 + measurementOffset), 3175.0);
        distanceMap.put(Units.feetToMeters(7.5 + measurementOffset), 3265.0);
        distanceMap.put(Units.feetToMeters(8 + measurementOffset), 3325.0);
        distanceMap.put(Units.feetToMeters(8.5 + measurementOffset), 3400.0);
        distanceMap.put(Units.feetToMeters(8.5 + measurementOffset), 3400.0);
        distanceMap.put(Units.feetToMeters(9 + measurementOffset), 3475.0);
        distanceMap.put(Units.feetToMeters(9.75 + measurementOffset), 3550.0);
        distanceMap.put(Units.feetToMeters(11.27 + measurementOffset), 4050.0);
    }

    /**
     * Finds the distance to current alliance hub
     * @return the distance in meters, -1 if no alliance is found;
     */
    private static double distanceToHub() {
        if (DriverStation.getAlliance().isPresent()) {
            Translation2d hub = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.Field.HUB_BLUE_TRANSLATION : Constants.Field.HUB_RED_TRANSLATION;
            Translation2d robot = Robot.swerveDrive.getPose().getTranslation();
            return robot.getDistance(hub);
        } else {
            return -1;
        }
    }

    private static double distanceToPass(){
         if (DriverStation.getAlliance().isPresent()) {
             double passline = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue? Constants.Field.PASS_LINE_BLUE_X : Constants.Field.PASS_LINE_RED_X;
             double robot = Robot.swerveDrive.getPose().getX();
             return Math.abs(passline - robot);
         }else {
            return -1;
         }
    }

    /**
     * This uses the pose estimate to the calculate the distance to the hub,
     * then it uses that distance to interpolate the ideal shooter speed from the
     * defined map.
     * @return The ideal speed of the shooter in RPM
     */
    public static double getShotRPM() {
        return distanceMap.get(distanceToHub());
    }

    public static double getPassRPM(){
        return distanceMap.get(distanceToPass());
    }

    /**
     * This uses the robot pose to calculate the
     * heading the robot needs to have to point toward the hub
     * @return The ideal rotation of the robot as a `Rotation2d`
     */
    private static Rotation2d angleToHub() {
        Translation2d robot = Robot.swerveDrive.getPose().getTranslation();
        Translation2d hub = Constants.Field.HUB_RED_TRANSLATION;

        if (DriverStation.getAlliance().isPresent()) {
            hub = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.Field.HUB_BLUE_TRANSLATION : Constants.Field.HUB_RED_TRANSLATION;
        }

        return hub.minus(robot).getAngle();
    }

    /**
     * Finds the error between the current robot angle
     * and the angle of the hub relative to the robot
     * @return angle as a Rotation2d
     */
    public static Rotation2d angleToHubError() {
        Rotation2d currentRotation = Robot.swerveDrive.getPose().getRotation();
        Rotation2d targetRot = optimizeAngle(angleToHub(), currentRotation);
        return new Rotation2d(targetRot.getRadians() - currentRotation.getRadians());
    }

    /**
     * Find the reverse of a given angle (i.e. pi/4->7pi/4)
     * @param radians the angle in radians to reverse
     * @return the reversed angle
     */
    private static double findRevAngle(double radians) {
        return (Math.PI * 2 + radians) % (2 * Math.PI) - Math.PI;
    }

    /**
     * Finds the distance in ticks between two setpoints
     * @param setpoint initial/current point
     * @param position desired position
     * @return the distance between the two point
     */
    private static double getDistance(double setpoint, double position) {
        return Math.abs(setpoint - position);
    }


    /**
     *  Chooses the shortest path to take to point at the hub
     * @param hubHeading - The heading of the hub relative to the robot
     * @param robotHeading - The current heading of the robot
     * @return The optimzied angle as a Rotation2d
     */
    private static Rotation2d optimizeAngle(Rotation2d hubHeading, Rotation2d robotHeading) {
        // Compute all options for a setpoint
        double position = robotHeading.getRadians();
        double setpoint = hubHeading.getRadians();
        double forward = setpoint + (2 * Math.PI);
        double reverse = setpoint - (2 * Math.PI);
        double antisetpoint = findRevAngle(setpoint);
        double antiforward = antisetpoint + (2 * Math.PI);
        double antireverse = antisetpoint - (2 * Math.PI);

        // Find setpoint option with minimum distance
        double[] alternatives = { forward, reverse};
        double min = setpoint;
        double minDistance = getDistance(setpoint, position);
        for (int i = 0; i < alternatives.length; i++) {
            double dist = getDistance(alternatives[i], position);
            if (dist < minDistance) {
                min = alternatives[i];
                minDistance = dist;
            }
        }

        return  new Rotation2d(min);
    }
}
