package team.gif.robot.commands.collector.collectorautos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

/**
 * Moves the collector down at a very high rate of speed
 * Used at the beginning of autonomous to deploy the collector fast
 */
public class CollectorAutonPivotDown extends Command {

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
        Robot.pivotMotor.runPivotPercent(1.0);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.pivotMotor.getPosition() > Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS * 0.90;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.setDefaultCommand(new CollectorAutoPivotHold());
    }
}
