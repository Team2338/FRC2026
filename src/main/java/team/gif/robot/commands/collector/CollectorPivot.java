package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorPivot extends Command {

    double percent;

    /**
     * Default command which constantly monitors the Aux joystick to move the colletor up and down
     * manually. Applies a small amount of power if the colletor is (close to being) fully deployed.
     */
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
        percent =  Math.min(percent, 0.3); // use a max value when going down/out
        if ((Robot.pivotMotor.getPosition() > Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS * 0.85) && percent >= 0){
            // apply small motor power to keep collector against the hard stop
            percent = Math.max(percent, 0.04);
        }
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
