package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPivotDown;
import team.gif.robot.commands.shooter.shooterautos.AutonShooterRPM;
import team.gif.robot.commands.shooter.shooterautos.IndexerAuton;

public class AutonCenterShoot extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos at the start of the match
     * from the Center shot. This sequence deploys the collector only part way in
     * order to keep the fuel close to the shooter. Also runs the collector to
     * move the fuel initially stored in the collector to the shooter.
     */
    public AutonCenterShoot() {
        super(new WaitCommand(4.0)); // max amount of time to run
        addCommands(
                new AutonShooterRPM(Constants.Shooter.SHOOTER_AUTON_CENTER_RPM),
                new WaitCommand(0.5).andThen(new CollectorAutoPivotDown(0.5).alongWith(new CollectorPercent(0.9).withTimeout(0.5)).andThen(new IndexerAuton(false))),
                new InstantCommand(() -> Robot.swerveDrive.stopModules()) // prevent the robot from shaking
        );
    }
}
