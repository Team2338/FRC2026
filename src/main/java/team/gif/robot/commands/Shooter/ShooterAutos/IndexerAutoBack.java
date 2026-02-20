package team.gif.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class IndexerAutoBack extends Command {
    double speed = 0;
    double seconds;
    double timer;

    public IndexerAutoBack(double seconds) {
        super();
        addRequirements(Robot.indexer);
        this.seconds = seconds;
        //addRequirements(Robot.climber); // uncomment
    }

    public IndexerAutoBack() {
        super();
        addRequirements(Robot.indexer);
        this.seconds = 999;
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //speed = SmartDashboard.getNumber("Indexer/Percent", 0);
        timer = 0;
        speed = 0.5;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.indexer.runPercent(-0.25, 0);
        timer++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return timer > 0.25 * 50;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopMotor();
    }
}
