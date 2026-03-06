package team.gif.robot.commands.collector.CollectorAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorAutoPivotDown extends Command {

    double percent;

    public CollectorAutoPivotDown(double percent) {
        super();
        addRequirements(Robot.pivotMotor);
        this.percent = percent;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.pivotMotor.runPivotPercent(percent);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.runPivotPercent(0);
    }
}
