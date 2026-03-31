package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ShooterDashboard extends Command {

    /**
     * Runs shooter motors at provided rpm, never self exits, stops the shooter motors when done
     * @param rpm rpm to run shooter
     */
    public ShooterDashboard(){
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double rpm = SmartDashboard.getNumber("Shooter/Reference", 0);
        Robot.shooter.runShooter(rpm);
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
