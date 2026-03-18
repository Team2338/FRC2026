package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorPercent extends Command {

    double percent = 0;

    public CollectorPercent() {
        super();
        addRequirements(Robot.collectMotor);
    }

    /**
     * Runs the collector at a provided percent, never ends
     *
     * @param perc positive value collects fuel, negative value ejects fuel
     */
    public CollectorPercent(double perc) {
        super();
        addRequirements(Robot.collectMotor);
        percent = perc;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.collectMotor.runCollectorPercent(-percent);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

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
