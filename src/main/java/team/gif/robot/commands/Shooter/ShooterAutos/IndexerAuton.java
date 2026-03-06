package team.gif.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerAuton extends Command {

    private int counter;
    private boolean initialRunReverse;

    /**
     * Full sequence to run indexer and agitator
     * @param runReverse set to true to initially run bottom indexer backward
     */
    public IndexerAuton(boolean runReverse) {
        super();
        addRequirements(Robot.indexer);
        this.initialRunReverse = runReverse;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialRunReverse = false;
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;
        if(initialRunReverse && counter < (Constants.Indexer.INDEXER_REVERSE_AUTO_SECONDS * 50)){
            Robot.indexer.run(-0.25, 0);
        }
        else {
            //Run the top indexer for a brief amount of time
            if (counter < (Constants.Indexer.INDEXER_REVERSE_TELEOP_SECONDS * 50)) {
                Robot.indexer.run(0, Constants.Indexer.TOP_INDEXER_MOTOR_PERCENT);
            } else {
                //Run the entire indexer and agitator
                Robot.indexer.run(Constants.Indexer.BOTTOM_INDEXER_MOTOR_PERCENT, Constants.Indexer.TOP_INDEXER_MOTOR_PERCENT);
                Robot.agitator.run(Constants.Collector.AGITATOR_MOTOR_AUTO_PERCENT);
            }
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
        Robot.indexer.stopMotors();
        Robot.agitator.stopMotor();
    }
}
