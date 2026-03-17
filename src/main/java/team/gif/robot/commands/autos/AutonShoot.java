package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoAgitate;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPercent;
import team.gif.robot.commands.shooter.shooterautos.IndexerAuton;
import team.gif.robot.commands.shooter.shooterautos.AutonShooterRPM;

public class AutonShoot extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos
     */
    public AutonShoot() {
        super(new AutonShooterRPM().withTimeout(6.0));
        addCommands(
                new WaitCommand(2.0).andThen(new CollectorAutoAgitate()),
                new WaitCommand(1.5).andThen(new IndexerAuton(true)),
                new CollectorAutoPercent(Constants.Collector.COLLECTOR_FAST_PERCENT)
        );
    }
}
