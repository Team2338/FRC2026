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
        distanceMap.put(Units.feetToMeters(15.0 + 0.75), 3750.0);
        distanceMap.put(Units.feetToMeters(13.42 + 0.75), 3650.0);
        distanceMap.put(Units.feetToMeters(11.42 + 0.75), 3500.0);
        distanceMap.put(Units.feetToMeters(8.42 + 0.75), 3100.0);
        distanceMap.put(Units.feetToMeters(7.42 + 0.75), 3050.0);
        distanceMap.put(Units.feetToMeters(6.42 + 0.75), 3000.0);
        distanceMap.put(Units.feetToMeters(5.42 + 0.75), 2750.0);
        distanceMap.put(Units.feetToMeters(4.42 + 0.75), 2650.0);
        distanceMap.put(Units.feetToMeters(3.42 + 0.75), 2550.0);
    }

    public static double distanceToHub() {
        if (DriverStation.getAlliance().isPresent()) {
            Translation2d hub = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.Field.HUB_BLUE_TRANSLATION : Constants.Field.HUB_RED_TRANSLATION;
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
