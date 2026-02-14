package team.gif.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class IndexerReverseManualMode extends Command {

    double stage1 = 0;
    double stage2 = 0;

    public IndexerReverseManualMode() {
        super();
        addRequirements(Robot.indexer);
        Robot.indexer.setIndexerManualMode(true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stage1 = SmartDashboard.getNumber("Indexer/Stage 1", 0);
        stage2 = SmartDashboard.getNumber("Indexer/Stage 2", 0);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.indexer.runPercent(-stage1, -stage2);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {Robot.indexer.stopMotor();}
}
