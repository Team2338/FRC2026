package team.gif.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerAutoBack extends Command {
    double seconds;
    double timer;

    public IndexerAutoBack(double seconds) {
        super();
        addRequirements(Robot.indexer);
        this.seconds = seconds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.indexer.runPercent(Constants.Indexer.INDEXER_BACK_PERCENT, 0);
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
        return timer >= (Constants.Indexer.INDEXER_BACK_AUTO_SECONDS * 50);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotor();
    }
}
