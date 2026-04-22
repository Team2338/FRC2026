package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPivotDown;
import team.gif.robot.commands.shooter.IndexerShoot;
import team.gif.robot.commands.shooter.ShooterRPM;

public class AutonStealShoot extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos at the start of the match
     * from the Center shot. This sequence deploys the collector only part way in
     * order to keep the fuel close to the shooter. Also runs the collector to
     * move the fuel initially stored in the collector to the shooter. Does not run agitator
     * wheels because there are only 8 fuel in the hopper and no need to run the agitator.
     */
    public AutonStealShoot() {
        super(new WaitCommand(4.0)); // max amount of time to run
        addCommands(
                // can not use shooterAuto and HubAutoALign because the collector is in the way of the cameras
                new ShooterRPM(3600),
                new CollectorAutoPivotDown(0.5).alongWith(new CollectorPercent(0.8).withTimeout(0.5)).andThen(new IndexerShoot().alongWith(new InstantCommand(()->Robot.agitator.setPercent(Constants.Collector.AGITATOR_MOTOR_PERCENT))))
        );
    }
}
