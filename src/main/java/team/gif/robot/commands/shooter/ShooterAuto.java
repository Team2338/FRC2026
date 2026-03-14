package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;
import team.gif.robot.commands.drivetrain.HubAutoAlign;
import team.gif.robot.subsystems.ShotCalculator;

public class ShooterAuto extends Command {
    /**
     * Runs shooter at RPM based distance to hub
     */


    public ShooterAuto() {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        Robot.shooter.runShooter(ShotCalculator.getShotRPM());

        if (Robot.shooter.isShooterReady() && (ShotCalculator.angleToHubError().getDegrees() <= 5)) {
            new IndexerPercent(0.8, 0.5);
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
        Robot.shooter.stopMotors();
    }
}
