package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class IndexerPercent extends Command {
    double bottomIndexerMotorPercent;
    double topIndexerMotorPercent;

    /**
     * Runs both indexer motors at provided percents, never self exits, stops indexer motors when done
     * @param topPercent percent to run top indexer (positive pulls fuel in)
     * @param bottomPercent percent to run the bottom indexer (positive collects)
     */
    public IndexerPercent(double bottomPercent, double topPercent) {
        super();
        addRequirements(Robot.indexer);
        bottomIndexerMotorPercent = bottomPercent;
        topIndexerMotorPercent = topPercent;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.indexer.runPercent(bottomIndexerMotorPercent, topIndexerMotorPercent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotors();
    }
}
