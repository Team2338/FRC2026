package team.gif.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AgitatorPercent extends Command {
    private double agitatorSpeed;

    /**
     * Runs the agitator wheels to pull fuel in at a default percentage, never self exits, stops the agitator wheels when done
     */
    public AgitatorPercent() {
        super();
        addRequirements(Robot.agitator);
        agitatorSpeed = Constants.Collector.AGITATOR_MOTOR_PERCENT;
    }

    /**
     * Runs tha agitator wheels at a provided percentage. <br>
     * Positive value rotates into Indexer, negative value rotates away from Indexer. <br>
     * Never self ends, stops the agitator wheels when done
     *
     * @param percent percent of power to run the agitator wheels
     */
    public AgitatorPercent(double percent) {
        super();
        addRequirements(Robot.agitator);
        agitatorSpeed = percent;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.agitator.setPercent(agitatorSpeed);
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
        Robot.agitator.stopMotor();
    }
}
