package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DriveSwerve extends Command {
    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;
    private final SlewRateLimiter turnLimiter;

    public DriveSwerve() {
        this.forwardLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.strafeLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.turnLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xPose = Robot.swerveDrive.getPoseX();
        double yPose = Robot.swerveDrive.getPoseY();
        
        if (Robot.diagnostics.anyMotorTempHot()) {
            if (Robot.isCompetition) {
                // For competitions, just print to console that motors are hot, but allow full functionality
                System.out.println("Motors are running hot.");
            } else {
                // For shop work, disable driving ability
                Robot.swerveDrive.stopDrive();
                System.out.println(Timer.getFPGATimestamp() + " Driving has been disabled. There is a motor which exceeds the safe temperature threshold.");
                return;
            }
        }

        double forwardSign;
        double strafeSign;

        double forward = -Robot.oi.driver.getLeftY(); // need to invert because -Y is away, +Y is pull back
        forward = (Math.abs(forward) > Constants.Joystick.DEADBAND) ? forward : 0.0; //0.00001;

        double strafe = -Robot.oi.driver.getLeftX(); // need to invert because -X is left, +X is right
        strafe = (Math.abs(strafe) > Constants.Joystick.DEADBAND) ? strafe : 0.0;

        double rot = -Robot.oi.driver.getRightX(); // need to invert because left is negative, right is positive
        rot = (Math.abs(rot) > Constants.Joystick.DEADBAND) ? rot : 0.0;

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

        //Align to fence while driving
        if (yPose >= Constants.Field.TOP_FENCE_ZONE_Y || yPose <= Constants.Field.BOTTOM_FENCE_ZONE_Y) {
            double targetAngle = Constants.Collector.FENCE_COLLECT_ANGLE;
            targetAngle =  (yPose >= Constants.Field.TOP_FENCE_ZONE_Y) ? targetAngle : -targetAngle;

            if (forward < 0.0) {
                targetAngle = (yPose >= Constants.Field.TOP_FENCE_ZONE_Y) ? 180 - targetAngle : -180 - targetAngle;
            }

            double angleError = ((targetAngle - Robot.pigeon.get180Heading()) + 180) % 360 - 180;
            rot = angleError * Constants.Mk5Constants.FENCE_ALIGN_P / 180;

        }
        else if(
                (xPose >= Constants.Field.HUB_ZONE_BLUE_MIN.getX() && yPose >= Constants.Field.HUB_ZONE_BLUE_MIN.getY()) &&
                (xPose <= Constants.Field.HUB_ZONE_BLUE_MAX.getX() && yPose <= Constants.Field.HUB_ZONE_BLUE_MAX.getY())){
            double targetAngle = Constants.Collector.HUB_COLLECT_ANGLE;

            if (strafe < 0.0) {
                targetAngle = -targetAngle;
            }
            else if (strafe > 0.0) {
                targetAngle = Math.abs(targetAngle);
            }

            double angleError = ((targetAngle - Robot.pigeon.get180Heading()) + 180) % 360 - 180;
            rot = angleError * Constants.Mk5Constants.FENCE_ALIGN_P / 180;
        }
        else if(
                (xPose >= Constants.Field.HUB_ZONE_RED_MIN.getX() && yPose >= Constants.Field.HUB_ZONE_RED_MIN.getY()) &&
                (xPose <= Constants.Field.HUB_ZONE_RED_MAX.getX() && yPose <= Constants.Field.HUB_ZONE_RED_MAX.getY())){
            double targetAngle = Constants.Collector.HUB_COLLECT_ANGLE;

            if (strafe < 0.0) {
                targetAngle = 180 - Math.abs(targetAngle);
            }
            else if (strafe > 0.0) {
                targetAngle = -(180 - Math.abs(targetAngle));
            }

            double angleError = ((targetAngle - Robot.pigeon.get180Heading()) + 180) % 360 - 180;
            rot = angleError * Constants.Mk5Constants.FENCE_ALIGN_P / 180;
        }
        else {
            // slow down the rotation by converting the linear response to a curve
            if (rot < 0 ) {
                rot = rot * -rot;
            } else {
                rot = rot * rot;
            }
        }

        rot = turnLimiter.calculate(rot) * Robot.swerveConfig.constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        Robot.swerveDrive.drive(forward, strafe, rot);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
