package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class HubCollect extends Command {
    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;
    private final SlewRateLimiter turnLimiter;
    private boolean isRedAlliance;
    private boolean forward;
    private int rotationTarget;
    private Twist2d collectorOffset;
    private double hubPosition;

    public HubCollect(boolean forward) {
        this.forwardLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.strafeLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.turnLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
        addRequirements(Robot.swerveDrive);

        this.forward = forward;
    }

    @Override
    public void initialize() {
        isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        Pose2d robotPose = Robot.swerveDrive.getPose();
        boolean isRightSide = robotPose.getX() < Constants.Field.CENTER_LINE;

        int baseAngle = robotPose.getY() < Constants.Field.FIELD_LENGTH / 2 ? 135 : 45;

        //Find collector edge pose
        double collectorExtension = Units.inchesToMeters(12);
        double robotYOffset = rotationTarget < 0 ? Robot.swerveConfig.constants.TRACK_WIDTH_METERS / 2 : -Robot.swerveConfig.constants.TRACK_WIDTH_METERS / 2;
        double robotXOffset = Robot.swerveConfig.constants.TRACK_LENGTH_METERS + collectorExtension;
        collectorOffset = new Twist2d(robotXOffset, robotYOffset, 0);


        hubPosition = robotPose.getY() < Constants.Field.FIELD_LENGTH / 2 ? Constants.Field.HUB_BLUE_REAR : Constants.Field.HUB_RED_REAR;



        rotationTarget = isRightSide ? -baseAngle : baseAngle;
        rotationTarget = isRedAlliance ? rotationTarget - 180 : rotationTarget;


    }

    @Override
    public void execute() {
        Pose2d robotPose = Robot.swerveDrive.getPose();


        double forwardSign;
        double strafeSign;

        double forward = -Robot.oi.driver.getLeftY(); // need to invert because -Y is away, +Y is pull back
        forward = (Math.abs(forward) > Constants.Joystick.DEADBAND) ? forward : 0.0; //0.00001;

        double strafe = -Robot.oi.driver.getLeftX(); // need to invert because -X is left, +X is right
        strafe = (Math.abs(strafe) > Constants.Joystick.DEADBAND) ? strafe : 0.0;

        forwardSign = forward/Math.abs(forward);
        strafeSign = strafe/Math.abs(strafe);
        // Use a parabolic curve (instead if linear) for the joystick to speed ratio
        // This allows for small joystick inputs to use slower speeds
        forward = Math.abs(forward) * forward;
        strafe = Math.abs(strafe) * strafe;

        forward = .5 * Math.sqrt(2 + forward*forward - strafe*strafe + 2*forward*Math.sqrt(2)) -
                .5 * Math.sqrt(2 + forward*forward - strafe*strafe - 2*forward*Math.sqrt(2));

        strafe = .5 * Math.sqrt(2 - forward*forward + strafe*strafe + 2*strafe*Math.sqrt(2)) -
                .5 * Math.sqrt(2 - forward*forward + strafe*strafe - 2*strafe*Math.sqrt(2));

        if( Double.isNaN(forward) )
            forward = forwardSign;
        if( Double.isNaN(strafe) )
            strafe = strafeSign;


        //Forward speed, Sideways speed, Rotation Speed
        forward = forwardLimiter.calculate(forward) * Robot.swerveDrive.getDrivePace().getValue();
        strafe = strafeLimiter.calculate(strafe) * Robot.swerveDrive.getDrivePace().getValue();


        //Find wall and set max speed based on distance
        Pose2d collectorPose = robotPose.exp(collectorOffset);
        double distanceToHub = Math.abs(collectorPose.getY() - hubPosition);
        double maxSpeed = (distanceToHub - Constants.Mk5Constants.AUTO_COLLECT_BUFFER) * 0.5; //1m away, maxSpeed = 2 m/s;

        strafe = Math.min(maxSpeed, strafe);

        // Auto rotate to 45 degrees
        double rotError = Robot.pigeon.getHeading() - rotationTarget;
        double rot = rotError * Constants.Mk5Constants.HUB_ALIGN_P / (2 * Math.PI); //Converts to -1 to 1 scale for limiter and speed calc

         rot = turnLimiter.calculate(rot) * Robot.swerveConfig.constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        // the robot starts facing the driver station so for this year negating y and x
        Robot.swerveDrive.drive(forward, strafe, rot);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
