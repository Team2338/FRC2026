package team.gif.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerAuton extends Command {

    private int counter;
    private boolean runBack;

    public IndexerAuton(boolean runBack) {
        super();
        addRequirements(Robot.indexer, Robot.preIndexer);
        this.runBack = runBack;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        runBack = false;
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;
//        if(Robot.shooter.isRPMSufficient(Constants.Shooter.SHOOTER_INITIAL_AUTON_RPM) && !alreadyRan) {
        if(runBack && counter < (Constants.Indexer.INDEXER_BACK_AUTO_SECONDS * 50)){
            Robot.indexer.runPercent(-0.25, 0);
        }
        else {
//            Run the stage 2 indexer for a certain amount of time
            if (counter < (Constants.Indexer.INDEXER_BACK_TELEOP_SECONDS * 50)) {
                Robot.indexer.runPercent(0, Constants.Indexer.INDEXER_STAGE_2_PERCENT);
            } else {
                //Run the entire indexer and agitator
                Robot.indexer.runPercent(Constants.Indexer.INDEXER_STAGE_1_PERCENT, Constants.Indexer.INDEXER_STAGE_2_PERCENT);
                Robot.preIndexer.runPercent(Constants.Indexer.INDEXER_STAGE_1_PERCENT);
                Robot.agitator.setPercent(Constants.Collector.AGITATOR_AUTO_PERCENT);
            }
        }
//        }
        //If the shooter RPM dips below the required RPM, then when it reaches the threshold do not run the indexer back
//        else if(Robot.shooter.isRPMSufficient(Constants.Shooter.SHOOTER_INITIAL_AUTON_RPM) && alreadyRan) {
//            Robot.indexer.runPercent(Constants.Indexer.INDEXER_STAGE_1_PERCENT, Constants.Indexer.INDEXER_STAGE_2_PERCENT);
//            Robot.preIndexer.runPercent(Constants.Indexer.INDEXER_STAGE_1_PERCENT);
//            Robot.agitator.setPercent(Constants.Collector.AGITATOR_AUTO_PERCENT);
//        }

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotor();
        Robot.preIndexer.stopMotor();
        Robot.agitator.stopMotor();
    }
}
