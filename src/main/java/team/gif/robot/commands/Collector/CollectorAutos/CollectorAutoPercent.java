package team.gif.robot.commands.Collector.CollectorAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorAutoPercent extends Command {

    double percent = 0;
    private int counter;

    public CollectorAutoPercent() {
        super();
        addRequirements(Robot.collectMotor);
    }

    public CollectorAutoPercent(double perc) {
        super();
        addRequirements(Robot.collectMotor);
        this.percent = perc;
    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
//        percent = SmartDashboard.getNumber("Collector/PID/Collect Percent", 0);
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
        return counter > Constants.Collector.COLLECTOR_CYCLE;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collectMotor.stopMotor();
    }
}
