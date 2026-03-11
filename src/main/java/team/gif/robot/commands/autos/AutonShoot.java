package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoAgitate;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoRun;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAuton;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRun;

public class AutonShoot extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos
     */
    public AutonShoot() {
        super(new ShooterAutoRun().withTimeout(6.0));
        addCommands(
                new CollectorAutoAgitate(),
                new WaitCommand(1.5).andThen(new IndexerAuton(true)),
                new CollectorAutoRun(Constants.Collector.COLLECTOR_FAST_RPM)
        );
    }
}
