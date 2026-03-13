package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.agitator.AgitatorAutonPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPercent;

public class CollectAutoSequence extends SequentialCommandGroup {
    /**
     * Not in use
     */
    public CollectAutoSequence() {
        addCommands(
                new ParallelCommandGroup(
                        new AgitatorAutonPercent(),
                        new CollectorAutoPercent(Constants.Collector.COLLECTOR_FAST_PERCENT)
                )
        );
    }

}
