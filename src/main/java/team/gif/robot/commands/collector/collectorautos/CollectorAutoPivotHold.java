package team.gif.robot.commands.collector.collectorautos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorAutoPivotHold extends Command {

    /**
     * Holds the collector down at a very small power. <br>
     * Utilized during autonomous to keep collector down while collecting fuel.
     */
    public CollectorAutoPivotHold() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.pivotMotor.runPivotPercent(0.04);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.runPivotPercent(0);
    }
}
