package team.gif.robot.commands.neo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class MoveReference extends Command {
    double speed;

    public MoveReference() {
        super();
        addRequirements(Robot.neo);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        speed = SmartDashboard.getNumber("PID/reference", 0);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
       Robot.neo.setReference(speed);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.neo.stop();
    }
}
