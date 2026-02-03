package team.gif.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ShooterRPM extends Command {
    double rpm = 0;

    public ShooterRPM() {
        super();
        addRequirements(Robot.shooter);
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rpm = SmartDashboard.getNumber("PID/Reference", 0);
        Robot.shooter.runShooter(rpm);

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
        Robot.shooter.stopMotor();
    }
}
