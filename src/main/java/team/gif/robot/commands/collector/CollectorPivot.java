package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorPivot extends Command {

    double percent;

    public CollectorPivot() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        percent = -Robot.oi.aux.getLeftY();
        percent = (Math.abs(percent) > Constants.Joystick.DEADBAND) ? percent : ((Robot.pivotMotor.getPosition() > Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS - 6) ? 0.1 : 0.0); //TODO: 6 is a placeholder, need to test the value
        percent *= Constants.Collector.PIVOT_PERCENT_MULTIPLIER;
        percent =  Math.min(percent, 0.2);
        Robot.pivotMotor.runPivotPercent(percent);

        if(Math.abs(percent) < Constants.Joystick.DEADBAND) {
            Robot.pivotMotor.holdPivotWithFF();
        }

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
