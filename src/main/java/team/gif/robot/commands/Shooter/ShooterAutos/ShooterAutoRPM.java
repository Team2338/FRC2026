package team.gif.robot.commands.Shooter.ShooterAutos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShooterAutoRPM extends Command {
    private int counter;

    public ShooterAutoRPM() {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.shooter.runShooter(Constants.Shooter.SHOOTER_INITIAL_AUTON_RPM);
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= (Constants.Shooter.SHOOTER_AUTO_SECONDS * 50);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.stopMotor();
    }
}
