package team.gif.robot.commands.Collector.CollectorAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorAutoPivotDown extends Command {

    double percent;

    public CollectorAutoPivotDown() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.pivotMotor.runPivotPercent(-0.8);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.pivotMotor.getPosition() >= Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.runPivotPercent(0);
    }
}
