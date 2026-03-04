package team.gif.robot.commands.Collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorRPM extends Command {

    private final double rpm;

    /**
     * @param rpm Positive is for collecting, negative for ejecting
     */
    public CollectorRPM(double rpm) {
        super();
        addRequirements(Robot.collectMotor);
        this.rpm = rpm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //dashRPM is meant for testing only, it should be removed after we have a good RPM value
        double dashRPM = SmartDashboard.getNumber("Collector/PID/Collect Reference", Constants.Collector.COLLECTOR_FAST_RPM);
        Robot.collectMotor.runCollector(dashRPM);
//        Robot.collectMotor.runCollector(rpm);
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
