package team.gif.robot.commands.Agitator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AgitatorAuto extends Command {

    public AgitatorAuto() {
        super();
        addRequirements(Robot.agitator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.agitator.setPercent(Constants.Collector.AGITATOR_AUTO_PERCENT);
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
