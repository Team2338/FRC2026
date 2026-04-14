package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.autos.AutonShoot;
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
        Robot.collectMotor.runCollectorPercent(0.8);
        Robot.agitator.setPercent(Constants.Collector.AGITATOR_MOTOR_PERCENT);
        Robot.shooter.runShooter(ShotCalculator.getPassRPM() + AutonShoot.shootCommand.getRPMOffset());

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

}