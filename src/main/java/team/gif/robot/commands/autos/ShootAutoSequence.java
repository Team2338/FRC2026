package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.shooter.shooterautos.IndexerAutoRunReverse;
import team.gif.robot.commands.shooter.shooterautos.IndexerAutoRun;
import team.gif.robot.commands.shooter.shooterautos.ShooterAutoRun;

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
