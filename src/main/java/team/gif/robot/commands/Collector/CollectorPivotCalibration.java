package team.gif.robot.commands.Collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorPivotCalibration extends Command {

    public CollectorPivotCalibration() {
        super();
        addRequirements(Robot.pivotMotor);
    }

    int counter;
    int time;
    int phase;
    double position;
    double positionTwo;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        phase = 0;
        time = 0;
        position = 0;
        positionTwo = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;

        /*
            Phase 0 - Run motor down until it stalls
                    - Then take motor position and transition to phase 1
            Phase 1 - Run motor up for set time
            Phase 2 - Repeat phase 0
                    - compare encoder readings, keep higher value
                    - transition to phase 3
            Phase 3 - end command
         */


        if (phase == 0 || phase == 2) {
            Robot.pivotMotor.runPivotPercent(Constants.Collector.PIVOT_CALIBRATION_PERCENT);
        } else if (phase == 1) {
            Robot.pivotMotor.runPivotPercent(-Constants.Collector.PIVOT_CALIBRATION_PERCENT);
        }

        if (Robot.pivotMotor.isStalling()){
            double encoder = Robot.pivotMotor.getPosition();
            position = Math.max(position, encoder);
            counter = 0;
            phase++;
        }

        if(phase == 1 && counter > 50 * 0.5) {
            counter = 0;
            phase++;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return phase == 3;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.pivotMotor.runPivotPercent(0);
        Robot.pivotMotor.deployedEncoder(position);
    }
}
