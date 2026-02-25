package team.gif.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerAutoPercent extends Command {
    double stage1 = 0;
    double stage2 = 0;
    private int counter;

    public IndexerAutoPercent() {
        super();
        addRequirements(Robot.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.indexer.runPercent(stage1, stage2);
        stage1 = 0.5; //Change value
        stage2 = 0.5; //Change value
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        counter++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= Constants.Shooter.SHOOTER_CYCLE;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotor();
    }
}
