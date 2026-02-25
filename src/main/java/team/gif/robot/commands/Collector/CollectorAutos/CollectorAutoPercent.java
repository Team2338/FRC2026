package team.gif.robot.commands.Collector.CollectorAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorAutoPercent extends Command {

    private final double percent;
    private int counter;

    public CollectorAutoPercent(double percent) {
        super();
        addRequirements(Robot.collectMotor);
        this.percent = percent;
    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        Robot.collectMotor.runCollectorPercent(-percent);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= (Constants.Collector.COLLECTOR_AUTO_SECONDS*50);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collectMotor.stopMotor();
    }
}
