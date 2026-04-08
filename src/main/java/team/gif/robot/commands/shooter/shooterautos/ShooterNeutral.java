package team.gif.robot.commands.shooter.shooterautos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShooterNeutral extends Command {

    private boolean inNeutral = false;

    public ShooterNeutral() {
        super();
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        inNeutral = Robot.swerveDrive.getPoseY() < Constants.Shooter.SHOOTER_AUTON_STOP_Y && Robot.pigeon.getRoll() < Constants.Shooter.SHOOTER_AUTON_STOP_PITCH;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return !inNeutral;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
