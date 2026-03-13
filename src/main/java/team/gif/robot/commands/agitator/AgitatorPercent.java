package team.gif.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AgitatorPercent extends Command {
    private double agitatorSpeed;

    public AgitatorPercent() {
        super();
        addRequirements(Robot.agitator);
        agitatorSpeed = Constants.Collector.AGITATOR_MOTOR_PERCENT;
    }

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
