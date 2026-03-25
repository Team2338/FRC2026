package team.gif.robot.commands.collector.collectorautos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorAutoAgitate extends Command {
    private int counter;
    private boolean isFinished;

    /**
     * Used to automatically agitate the collector up and down during shooting
     */
    public CollectorAutoAgitate() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        // do not execute auto sequence if collector is not deployed
        isFinished = Robot.pivotMotor.getPosition() < Constants.Collector.PIVOT_LIMIT_AUTO_AGITATE_START_POS;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // abort sequence if during initialize it was determined not to run
        if (isFinished){
            return;
        }

        // Cycle 1
        if(counter == (int)(0.0 * 50)) {
            Robot.pivotMotor.runPivotPercent(-0.5);
        }
        if(counter == (int)(0.3 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        if(counter == (int)(0.4 * 50)) {
            Robot.pivotMotor.runPivotPercent(0.3);
        }
        if(counter == (int)(0.6 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        // Cycle 2
        if(counter == (int)(1.0 * 50)) {
            Robot.pivotMotor.runPivotPercent(-0.5);
        }
        if(counter == (int)(1.3 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        if(counter == (int)(1.4 * 50)) {
            Robot.pivotMotor.runPivotPercent(0.3);
        }
        if(counter == (int)(1.8 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        // Cycle 3
        if(counter == (int)(2.1 * 50)) {
            Robot.pivotMotor.runPivotPercent(-0.5);
        }
        if(counter == (int)(2.3 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        if(counter == (int)(2.4 * 50)) {
            Robot.pivotMotor.runPivotPercent(0.3);
        }
        if(counter == (int)(2.6 * 50)) {
            Robot.pivotMotor.stopMotor();
            isFinished = true; // set to true to indicate full sequence is complete
        }
        counter++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.stopMotor();
    }
}
