package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorRun extends Command {

    private double output;

    /**
     * @param desiredOutput Positive is for collecting, negative for ejecting
     */
    public CollectorRun(double desiredOutput) {
        super();
        addRequirements(Robot.collectMotor);
        output = desiredOutput;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.collectMotor.run(output);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        //Changes output if the output is changed in dashboard
        double newOutput = SmartDashboard.getNumber("Collector/Desired Output", output);
        if (output != newOutput) {
            output = newOutput;
            Robot.collectMotor.run(output);
        }
    }

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
