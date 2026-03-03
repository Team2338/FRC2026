package team.gif.robot.commands.Collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorRPM extends Command {

    double rpm = 0;

    /**
     * Not in use
     */
    public CollectorRPM() {
        super();
        addRequirements(Robot.collectMotor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rpm = SmartDashboard.getNumber("Collector/PID/Collect Reference", 0);
        Robot.collectMotor.runCollector(rpm);
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
        Robot.collectMotor.stopMotor();
    }
}
