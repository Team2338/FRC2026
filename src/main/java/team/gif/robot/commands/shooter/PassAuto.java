package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.ShotCalculator;

public class PassAuto extends Command {
    private boolean readyToIndex;
    private double rpmOffset = 0;
    private boolean sequenceScheduled;
    private boolean manualMode = false; // not directly used here but related to Auto shooting
    private Command indexerCommand;

    public PassAuto() {
        super();
        addRequirements(Robot.shooter);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        readyToIndex = false;
        sequenceScheduled = false;
        indexerCommand = Constants.Indexer.getFullCommand();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.shooter.runShooter(ShotCalculator.getPassRPM() + rpmOffset);

        if (Robot.shooter.getShooterReady()) {
            readyToIndex = true;
        }

        if (!sequenceScheduled && readyToIndex) {
            CommandScheduler.getInstance().schedule(indexerCommand);
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
        Robot.shooter.stopMotors(); // added because teleop would start with shooter motor running
        indexerCommand.cancel();
        sequenceScheduled = false;
    }

    /**
     * Adjusts the offset to the calculated required RPM
     *
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
     *
     * @return offset in rpm
     */
    public double getRPMOffset() {
        return rpmOffset;
    }

    /**
     * Sets manual mode to true/false
     *
     * @param mode true when in manual mode
     */
    public void setManualMode(boolean mode) {
        manualMode = mode;
    }

    /**
     * Gets the current shooter manual mode
     *
     * @return true if manual mode is enabled
     */
    public boolean getManualMode() {
        return manualMode;
    }

}