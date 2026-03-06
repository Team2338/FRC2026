package team.gif.robot.commands.shooter.ShooterAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerAutonReverse extends Command {
    double seconds;
    double timer;
    /**
     * Not in use
     */
    public IndexerAutonReverse(double seconds) {
        super();
        addRequirements(Robot.indexer);
        this.seconds = seconds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.indexer.runPercent(Constants.Indexer.INDEXER_REVERSE_PERCENT, 0);
        timer = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        timer++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return timer >= (Constants.Indexer.INDEXER_REVERSE_AUTO_SECONDS * 50);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotors();
    }
}
