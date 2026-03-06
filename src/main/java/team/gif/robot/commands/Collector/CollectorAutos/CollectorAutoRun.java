package team.gif.robot.commands.Collector.CollectorAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorAutoRun extends Command {
    private double output;

    /**
     * @param desiredOutput Positive is for collecting, negative is for ejecting
     */
    public CollectorAutoRun(double desiredOutput) {
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
