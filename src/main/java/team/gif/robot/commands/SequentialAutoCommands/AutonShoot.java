package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.Agitator.AgitatorAuto;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPercent;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAuton;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRPM;

public class AutonShoot extends ParallelCommandGroup {

    public AutonShoot() {
        addCommands(
                new ShooterAutoRPM(),
                new CollectorAutoPercent(Constants.Collector.COLLECTOR_FAST_PERCENT, true),
                new AgitatorAuto(),
                new IndexerAuton()
        );
    }
}
