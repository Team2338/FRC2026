package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.commands.autos.AutonShoot;

public class ToggleShooterManualMode extends Command {

    /**
     * Enables manual mode in initialize() and disables manual mode in end() <br>
     * Useful in OI to toggle on the state of manual mode. <br>
     * isFinished returns false, otherwise when initially toggled, it will immediately end and
     * then call end() which will disable manual mode
     */
    public ToggleShooterManualMode() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        AutonShoot.shootCommand.setManualMode(true);
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
        AutonShoot.shootCommand.setManualMode(false);
    }
}
