package team.gif.robot.commands.drivetrain.collectalign;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class HubCollectAlignLeft extends Command {
    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;

    public HubCollectAlignLeft() {
        this.forwardLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.strafeLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
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

        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        double targetAngle = alliance == DriverStation.Alliance.Red ? Constants.Collector.HUB_COLLECT_ANGLE : -Constants.Collector.FENCE_COLLECT_ANGLE;

        double angleError = Math.IEEEremainder(targetAngle, 360.0);
        rot = angleError * Constants.Mk5Constants.HUB_COLLECT_ALIGN_P;

        Robot.swerveDrive.drive(forward, strafe, rot);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
