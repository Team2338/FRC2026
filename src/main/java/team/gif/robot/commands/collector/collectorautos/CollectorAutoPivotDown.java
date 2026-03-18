package team.gif.robot.commands.collector.collectorautos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorAutoPivotDown extends Command {
    private double downPos = 0.95;

    public CollectorAutoPivotDown() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    public CollectorAutoPivotDown(double position) {
        downPos = position;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent;

        // use a ratio of the % distance remaining
        percent = 0.6 * (Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS - Robot.pivotMotor.getPosition())/Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS;

        // set a minimum value to move the pivot motor
        percent = Math.max(percent,0.2);
        System.out.println(Timer.getFPGATimestamp() + " percent " + percent);

        Robot.pivotMotor.runPivotPercent(percent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // stop when the pivot is close to fully deployed (and then let the default command pull it down the rest of the way)
        return Robot.pivotMotor.getPosition() > Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS * downPos;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // the default command takes over and forces the collector down the rest fo the way
    }
}
