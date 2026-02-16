package team.gif.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShotCalculator {
    private final static InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

    static {
        distanceMap.put(Units.feetToMeters(11.42), 3500.);
        distanceMap.put(Units.feetToMeters(13.42), 3650.);
        distanceMap.put(Units.feetToMeters(15), 3750.);
        distanceMap.put(Units.feetToMeters(8.42), 3100.);
        distanceMap.put(Units.feetToMeters(7.42), 3050.);
        distanceMap.put(Units.feetToMeters(6.42), 3000.);
        distanceMap.put(Units.feetToMeters(5.42), 2750.);
        distanceMap.put(Units.feetToMeters(4.42), 2650.);
        distanceMap.put(Units.feetToMeters(3.42), 2550.);

    }

    public static double distanceToHub() {
        if (DriverStation.getAlliance().isPresent()) {
            Translation2d hub = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.Field.HUB_BLUE : Constants.Field.HUB_RED;
            Translation2d robot = Robot.swerveDrive.getPose().getTranslation();
            return robot.getDistance(hub);
        } else {
            return -1;
        }
    }

    public static double getShotRPM() {
        return distanceMap.get(distanceToHub());
    }

}
