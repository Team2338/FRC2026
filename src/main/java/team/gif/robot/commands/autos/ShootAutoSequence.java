package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.shooter.ShooterAutos.IndexerAutonReverse;
import team.gif.robot.commands.shooter.ShooterAutos.IndexerAutoPercent;
import team.gif.robot.commands.shooter.ShooterAutos.AutonShooterRPM;

public class ShootAutoSequence extends SequentialCommandGroup {
    /**
     * Not in use
     */
    public ShootAutoSequence() {
        addCommands(
               new IndexerAutonReverse(Constants.Indexer.INDEXER_REVERSE_TELEOP_SECONDS),
                new ParallelCommandGroup(
                        new IndexerAutoPercent(),
                        new AutonShooterRPM()
                )
        );
    }

}
