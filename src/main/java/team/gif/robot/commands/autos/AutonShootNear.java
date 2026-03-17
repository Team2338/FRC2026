package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoAgitate;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPercent;
import team.gif.robot.commands.shooter.shooterautos.AutonShooterRPM;
import team.gif.robot.commands.shooter.shooterautos.IndexerAuton;

public class AutonShootNear extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos
     */
    public AutonShootNear() {
        super(new AutonShooterRPM(Constants.Shooter.SHOOTER_AUTON_NEAR_RPM).withTimeout(3.0));
        addCommands(
                new WaitCommand(0.25).andThen(new IndexerAuton(true)),
                new WaitCommand(0.75).andThen(new CollectorAutoAgitate()),
                new CollectorAutoPercent(Constants.Collector.COLLECTOR_FAST_PERCENT)
        );
    }
}
