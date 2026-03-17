package team.gif.robot.commands.collector.collectorautos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorAgitateOnce extends Command {
    private int counter;

    /**
     * Used to automatically agitate the collector up 1 time
     */
    public CollectorAgitateOnce() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // only agitate of above threshold to not over extend the collector
        if (Robot.pivotMotor.getPosition() > Constants.Collector.PIVOT_LIMIT_AGITATE_POS) {
            Robot.pivotMotor.runPivotPercent(-0.5);
        } else {
            Robot.pivotMotor.stopMotor();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter++ > .20 * 50;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.stopMotor();
    }
}
