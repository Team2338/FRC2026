package team.gif.robot.commands.shooter.shooterautos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.shooter.ShooterAuto;

public class IndexerAuton extends Command {

    Command commandSequence;

    /**
     * Full sequence to run indexer and agitator wheels during autonomous
     * @param runReverse set to true to initially run bottom indexer backward
     */
    public IndexerAuton(boolean runReverse) {
        super();
        addRequirements(Robot.indexer);
        addRequirements(Robot.agitator);
        commandSequence = runReverse ? ShooterAuto.indexFullCommand : ShooterAuto.indexForwardCommand;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(commandSequence);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.agitator.setPercent(Constants.Collector.AGITATOR_MOTOR_PERCENT);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        commandSequence.cancel();
        Robot.agitator.stopMotor();
    }
}
