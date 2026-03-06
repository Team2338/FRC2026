package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAutoRunReverse;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAutoRun;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRun;

public class ShootAutoSequence extends SequentialCommandGroup {
    /**
     * Not in use
     */
    public ShootAutoSequence() {
        addCommands(
               new IndexerAutoRunReverse(Constants.Indexer.INDEXER_REVERSE_TELEOP_SECONDS),
                new ParallelCommandGroup(
                        new IndexerAutoRun(),
                        new ShooterAutoRun()
                )
        );
    }

}
