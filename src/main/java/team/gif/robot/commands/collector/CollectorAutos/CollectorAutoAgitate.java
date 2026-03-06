package team.gif.robot.commands.collector.CollectorAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorAutoAgitate extends Command {
    private int counter;

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
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;
        // Cycle 1
        if(counter == (int)(2.0 * 50)) {
            Robot.pivotMotor.runPivotPercent(-0.7);
        }
        if(counter == (int)(2.3 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        if(counter == (int)(2.4 * 50)) {
            Robot.pivotMotor.runPivotPercent(0.3);
        }
        if(counter == (int)(2.6 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        // Cycle 2
        if(counter == (int)(3.0 * 50)) {
            Robot.pivotMotor.runPivotPercent(-0.5);
        }
        if(counter == (int)(3.3 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        if(counter == (int)(3.4 * 50)) {
            Robot.pivotMotor.runPivotPercent(0.3);
        }
        if(counter == (int)(3.8 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        // Cycle 3
        if(counter == (int)(4.1 * 50)) {
            Robot.pivotMotor.runPivotPercent(-0.5);
        }
        if(counter == (int)(4.3 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
        if(counter == (int)(4.4 * 50)) {
            Robot.pivotMotor.runPivotPercent(0.3);
        }
        if(counter == (int)(4.6 * 50)) {
            Robot.pivotMotor.stopMotor();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.stopMotor();
    }
}
