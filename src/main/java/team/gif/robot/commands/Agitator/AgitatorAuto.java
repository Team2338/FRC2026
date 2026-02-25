package team.gif.robot.commands.Agitator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AgitatorAuto extends Command {
    private int counter;

    public AgitatorAuto() {
        super();
        addRequirements(Robot.agitator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        Robot.agitator.setPercent(-0.3);

    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= Constants.Collector.AGITATOR_CYCLE;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.agitator.stopMotor();
    }
}
