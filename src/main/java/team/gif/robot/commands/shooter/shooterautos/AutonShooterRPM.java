package team.gif.robot.commands.shooter.shooterautos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShooterAutoRun extends Command {
    /**
     * Runs shooter at the auton RPM value
     */
    public ShooterAutoRun() {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.shooter.run(Constants.Shooter.SHOOTER_INITIAL_AUTON_RPM);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.stopMotors();
    }
}
