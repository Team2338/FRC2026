package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorPivot extends Command {

    double percent;

    public CollectorPivot() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        percent = -Robot.oi.aux.getLeftY(); // get joystick position (Left stick forward is negative)
        percent = (Math.abs(percent) > Constants.Joystick.DEADBAND) ? percent : 0.0; // ignore deadband
        percent *= Constants.Collector.PIVOT_PERCENT_MULTIPLIER; // 100% is too fast so slow it down

        // positive value is down
        percent =  Math.min(percent, 0.3); // use a max value of 20% when gown down/out
//        if ((Robot.pivotMotor.getPosition() > Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS * 0.8) && percent >= 0){
//        if ( Robot.shooter.getLeftMotorSpeed() < 500 & percent != 0) {
//            percent = Math.max(percent, 0.03);
//        }
        Robot.pivotMotor.runPivotPercent(percent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
