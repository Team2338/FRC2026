package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class IndexerPercent extends Command {
    double bottomIndexerMotorPercent = 0;
    double topIndexerMotorPercent = 0;

    /**
     * Runs both indexer motors at given percents
     */
    public IndexerPercent() {
        super();
        addRequirements(Robot.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bottomIndexerMotorPercent = 0.5;
        topIndexerMotorPercent = 0.8;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.indexer.runPercent(topIndexerMotorPercent, bottomIndexerMotorPercent);
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
