package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ReverseCollectorPercent extends Command {

    double percent = 0;

    /**
     * Runs the collector in reverse at provided percentage, never self exits. Stops the collector motor when done.
     *
     * @param perc percentage to run collector. Value is 0 to 1. Positive value ejects fuel.
     */
    public ReverseCollectorPercent() {
        super();
        addRequirements(Robot.collectMotor);
        percent = 1.0;
    }

    public ReverseCollectorPercent(double perc) {
        super();
        addRequirements(Robot.collectMotor);
        percent = perc;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.collectMotor.runCollectorPercent(percent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collectMotor.stopMotor();
    }
}
