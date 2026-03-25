package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class IndexerReverse extends Command {
    double secondsToRun;
    double timer;
    double percent;

    /**
     * Runs the bottom indexer at x percent for y seconds
     * @param percent percent to run the bottom indexer (positive value is eject)
     * @param seconds number of seconds to run indexer
     */
    public IndexerReverse(double percent, double seconds) {
        super();
        addRequirements(Robot.indexer);
        secondsToRun = seconds;
        this.percent = percent;
    }

    /**
     * Runs the bottom indexer at a provided percent, never self exits, stops both indexer motors when done
     */
    public IndexerReverse(double percent) {
        super();
        addRequirements(Robot.indexer);
        secondsToRun = Double.MAX_VALUE;
        this.percent = percent;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.indexer.runPercent(-percent, 0);
        timer++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return timer > secondsToRun * 50;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotors();
    }
}
