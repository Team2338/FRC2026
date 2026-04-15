package team.gif.robot.commands.collector.collectorautos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorAutonPivotDown extends Command {
    /**
     * Moves the collector down at a very high rate of speed. Slows the speed down when close to
     * being fully deployed. Holds the collector down.
     * Used at the beginning of autonomous to deploy the collector fast.
     */
    public CollectorAutonPivotDown() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.pivotMotor.getPosition() < Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS * 0.9) {
            Robot.pivotMotor.runPivotPercent(1.0);
        } else {
            Robot.pivotMotor.runPivotPercent(Constants.Collector.PIVOT_DOWN_HOLD_PERCENT);
        }
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
