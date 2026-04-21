package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.Robot;

public class DetectBeach extends Command {
    private boolean beaching;
    /**
     * Not in use
     */
    public DetectBeach() {
//        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        beaching = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (beaching) {
            //can't do this bc of requirements
            //Robot.swerveDrive.stopDrive();
        }

        if(Robot.pigeon.getRoll() > 5 || Robot.pigeon.getPitch() > 5) {
            beaching = true;
            System.out.println("******");
            System.out.println("******");
            System.out.println("******");
            System.out.println("******");
            System.out.println("******");
            System.out.println("******");
            DriverStation.reportWarning("Beach detected in auto - Roll: " + Robot.pigeon.getRoll() + " Pitch: " + Robot.pigeon.getPitch(), false);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return beaching;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (beaching) {
            Robot.cancelAutonomousCommand();
            CommandScheduler.getInstance().schedule(new StopModules());
        }
    }
}