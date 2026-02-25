package team.gif.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class PreIndexerAutoPercent extends Command {

    double percent;
    private int counter;

    public PreIndexerAutoPercent() {
        super();
        addRequirements(Robot.preIndexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        percent = 0.5; //Change value
        Robot.preIndexer.runPercent(percent);
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
        Robot.preIndexer.stopMotor();
    }
}
