package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoAgitate;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoRPM;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAuton;
import team.gif.robot.commands.Shooter.ShooterAutos.AutonShooterRPM;

public class AutonShoot extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos
     */
    public AutonShoot() {
        super(new AutonShooterRPM().withTimeout(6.0));
        addCommands(
                new CollectorAutoAgitate(),
                new WaitCommand(1.5).andThen(new IndexerAuton(true)),
                new CollectorAutoRPM(Constants.Collector.COLLECTOR_FAST_RPM)
        );
    }
}
