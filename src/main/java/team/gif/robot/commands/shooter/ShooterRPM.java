package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;
import team.gif.robot.commands.autos.AutonShoot;

public class ShooterRPM extends Command {
    double rpm;

    /**
     * Runs shooter motors at provided rpm, never self exits, stops the shooter motors when done
     * @param rpm rpm to run shooter
     */
    public ShooterRPM(double rpm){
        super();
        addRequirements(Robot.shooter);
        this.rpm = rpm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.shooter.runShooter(rpm + AutonShoot.shootCommand.getRPMOffset());
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
