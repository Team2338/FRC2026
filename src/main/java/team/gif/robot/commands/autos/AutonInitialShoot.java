package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPivotDown;
import team.gif.robot.commands.shooter.shooterautos.IndexerAuton;
import team.gif.robot.commands.shooter.shooterautos.AutonShooterRPM;

public class AutonInitialShoot extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos at the start of the match
     * with the collector up
     */
    public AutonInitialShoot() {
        super(new AutonShooterRPM().withTimeout(Constants.Shooter.SHOOTER_AUTO_SECONDS));
        addCommands(
                new CollectorAutoPivotDown().withTimeout(1.5).andThen(new CollectorAutoPivotDown()),
                new WaitCommand(1.5).andThen(new IndexerAuton(false))
        );
    }
}
