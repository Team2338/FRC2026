package team.gif.robot.commands.collector.collectorautos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorAutoPivotDown extends Command {
    private double downPosPerc = 0.95;

    /**
     * Moves the collector down/out to given % of deployed position
     *
     * Does not stop the collector. Lets the default command pull
     * the collector down the rest of the way.
     */
    public CollectorAutoPivotDown() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    /**
     * Moves the collector down/out to provided % of deployed position
     *
     * Does not stop the collector. Lets the default command pull
     * the collector down the rest of the way.
     * @param percentTotalDown % of the deployed position
     */
    public CollectorAutoPivotDown(double percentTotalDown) {
        super();
        addRequirements(Robot.pivotMotor);
        downPosPerc = percentTotalDown;
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

        Robot.pivotMotor.runPivotPercent(percent);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // stop when the pivot is close to fully deployed (and then let the default command pull it down the rest of the way)
        return Robot.pivotMotor.getPosition() > Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS * downPosPerc;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.stopMotor();
        // the default command takes over and forces the collector down the rest fo the way
    }
}
