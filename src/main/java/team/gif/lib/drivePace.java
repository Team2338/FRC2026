package team.gif.lib;

import team.gif.robot.Robot;

public enum drivePace {
    COAST_FR(Robot.swerveConfig.constants.COAST_SPEED_METERS_PER_SECOND, true),
    COAST_RR(Robot.swerveConfig.constants.COAST_SPEED_METERS_PER_SECOND, false),
    COAST_ROT(Robot.swerveConfig.constants.COAST_SPEED_METERS_PER_SECOND, false),
//    SLOW_FR(Constants.Drivetrain.SLOW_SPEED_METERS_PER_SECOND, true),
//    SLOW_RR(Constants.Drivetrain.SLOW_SPEED_METERS_PER_SECOND, false),
    BOOST_FR(Robot.swerveConfig.constants.BOOST_SPEED_METERS_PER_SECOND, true);
//    BOOST_RR(Constants.Drivetrain.BOOST_SPEED_METERS_PER_SECOND, false);

    private final double value;
    private final boolean isFieldRelative;

    drivePace(double value, boolean isFieldRelative) {
        this.value = value;
        this.isFieldRelative = isFieldRelative;
    }

    public double getValue() {
        return this.value;
    }
    public boolean getIsFieldRelative() {
        return this.isFieldRelative;
    }
}
