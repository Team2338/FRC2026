package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.ShotCalculator;

public class ShooterAuto extends Command {
    private boolean readyToIndex;
    private static final Command indexCommand = new IndexerReverse(0.75, 0.25).andThen(new IndexerPercent(0.8, 0.5));
    private double rpmOffset = 0;

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
        readyToIndex = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        Robot.shooter.runShooter(ShotCalculator.getShotRPM() + rpmOffset);

        if (Robot.shooter.getShooterReady() && (ShotCalculator.angleToHubError().getDegrees() <= 3)) {
            readyToIndex = true;
        }

        if (readyToIndex) {
            CommandScheduler.getInstance().schedule(indexCommand);
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
        indexCommand.cancel();
    }

    /**
     * Adjusts the offset to the calculated required RPM
     * @param rpm - desired change in rpm
     */
    public void adjustRPMOffset(double rpm) {
        rpmOffset = rpmOffset + rpm;
    }

    /**
     * Sets the rpm offset to zero
     */
    public void resetRPMOffset() {
        rpmOffset = 0;
    }
}
