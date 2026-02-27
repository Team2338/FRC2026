package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.Agitator.AgitatorAuto;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoAgitate;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoCollectRPM;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPercent;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPivotDown;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAuton;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRPM;

public class AutonShoot extends ParallelDeadlineGroup {

    public AutonShoot() {
        super(new ShooterAutoRPM().withTimeout(6.0));
        addCommands(
//                new AgitatorAuto(),
                new CollectorAutoAgitate(),
                new WaitCommand(1.5).andThen(new IndexerAuton(true)),
                new CollectorAutoPercent(Constants.Collector.COLLECTOR_FAST_PERCENT)
        );
    }
}
