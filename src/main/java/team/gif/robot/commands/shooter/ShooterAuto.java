package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.ShotCalculator;

public class ShooterAuto extends Command {
    public static final Command indexReverseCommand = new IndexerPercent(-0.75, 0.0).withTimeout(0.17).andThen(new IndexerPercent(-0.75, 1.0).withTimeout(0.07));
    public static final Command indexForwardCommand = new IndexerShoot();
    public static final Command indexFullCommand = indexReverseCommand.andThen(indexForwardCommand);

    private boolean readyToIndex;
    private double rpmOffset = 0;
    private boolean sequenceScheduled;

    /**
     * Revs shooter and shoots fuel (i.e. runs indexer motors) when shooter reaches minimum RPM.<br>
     * Robot must be within angle tolerance of seeing the hub.<br>
     * Never self exists, stops all shooter and both indexer wheels when done.
     */
    public ShooterAuto() {
        super();
        addRequirements(Robot.shooter);
        addRequirements(Robot.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        readyToIndex = false;
        sequenceScheduled = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.shooter.runShooter(ShotCalculator.getShotRPM() + rpmOffset);

        if (Robot.shooter.getShooterReady() && (ShotCalculator.angleToHubError().getDegrees() <= 3)) {
            readyToIndex = true;
        }

        // only want to run this if shooter is ready, bot is aligned, and it hasn't been run already
        // i.e. only schedule the index sequence once
        if (!sequenceScheduled && readyToIndex) {
            CommandScheduler.getInstance().schedule(indexFullCommand);
            sequenceScheduled = true;
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
        indexFullCommand.cancel();
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

    /**
     * Returns the rpm offset
     * @return offset in rpm
     */
    public double getRPMOffset() {
        return rpmOffset;
    }
}
