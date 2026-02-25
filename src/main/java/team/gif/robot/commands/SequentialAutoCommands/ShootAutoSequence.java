package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.Agitator.AgitatorAuto;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPercent;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPivotDown;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAutoBack;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAutoPercent;
import team.gif.robot.commands.Shooter.ShooterAutos.PreIndexerAutoPercent;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRPM;

import javax.naming.PartialResultException;

public class ShootAutoSequence extends SequentialCommandGroup {
    public ShootAutoSequence() {
        addCommands(
               new IndexerAutoBack(Constants.Indexer.INDEXER_BACK_SECONDS),
                new ParallelCommandGroup(
                        new IndexerAutoPercent(),
                        new PreIndexerAutoPercent(),
                        new ShooterAutoRPM()
                )
        );
    }

}
