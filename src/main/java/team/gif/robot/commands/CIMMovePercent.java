package team.gif.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CIMMovePercent extends Command {
    double speed;

    public CIMMovePercent() {
        super();
        addRequirements(Robot.cim);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        speed = SmartDashboard.getNumber("PID/CIM Percent", 0);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
       Robot.cim.percent(speed);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.cim.stop();
    }
}
