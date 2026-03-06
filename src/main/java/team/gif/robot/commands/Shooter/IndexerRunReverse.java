package team.gif.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerRunReverse extends Command {
    double seconds;
    double timer;

    /**
     * Runs the bottom indexer at a given RPM for x seconds
     * @param seconds number of seconds to run indexer
     */
    public IndexerRunReverse(double seconds) {
        super();
        addRequirements(Robot.indexer);
        this.seconds = seconds;
    }

    /**
     * Runs the bottom indexer at a given RPM indefinitely
     */
    public IndexerRunReverse() {
        super();
        addRequirements(Robot.indexer);
        this.seconds = 999;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.indexer.run(Constants.Indexer.INDEXER_REVERSE_PERCENT, 0);
        timer++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return timer > seconds * 50;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotors();
    }
}
