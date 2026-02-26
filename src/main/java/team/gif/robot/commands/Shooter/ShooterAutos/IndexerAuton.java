package team.gif.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerAuton extends Command {

    private int counter;
    private boolean alreadyRan;

    public IndexerAuton() {
        super();
        addRequirements(Robot.indexer, Robot.preIndexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        alreadyRan = false;
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if(Robot.shooter.isRPMSufficient(Constants.Shooter.SHOOTER_INITIAL_AUTON_RPM) && !alreadyRan) {

            //Run the indexer back for a certain amount of time
            while(counter < (Constants.Indexer.INDEXER_BACK_TELEOP_SECONDS * 50)) {
                counter++;
                Robot.indexer.runPercent(Constants.Indexer.INDEXER_BACK_PERCENT, 0);
            }

            Robot.indexer.runPercent(Constants.Indexer.INDEXER_STAGE_1_PERCENT, Constants.Indexer.INDEXER_STAGE_2_PERCENT);
            Robot.preIndexer.runPercent(Constants.Indexer.INDEXER_STAGE_1_PERCENT);
        }
        //If the shooter RPM dips below the required RPM, then when it reaches the threshold do not run the indexer back
        else if(Robot.shooter.isRPMSufficient(Constants.Shooter.SHOOTER_INITIAL_AUTON_RPM) && alreadyRan) {
            Robot.indexer.runPercent(Constants.Indexer.INDEXER_STAGE_1_PERCENT, Constants.Indexer.INDEXER_STAGE_2_PERCENT);
            Robot.preIndexer.runPercent(Constants.Indexer.INDEXER_STAGE_1_PERCENT);
        }

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= (Constants.Indexer.INDEXER_AUTON_SECONDS * 50);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotor();
        Robot.preIndexer.stopMotor();
    }
}
