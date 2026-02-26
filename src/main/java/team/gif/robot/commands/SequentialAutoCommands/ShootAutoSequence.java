package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAutoBack;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAutoPercent;
import team.gif.robot.commands.Shooter.ShooterAutos.PreIndexerAutoPercent;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRPM;

public class ShootAutoSequence extends SequentialCommandGroup {
    public ShootAutoSequence() {
        addCommands(
               new IndexerAutoBack(Constants.Indexer.INDEXER_BACK_TELEOP_SECONDS),
                new ParallelCommandGroup(
                        new IndexerAutoPercent(),
                        new PreIndexerAutoPercent(),
                        new ShooterAutoRPM()
                )
        );
    }

}
