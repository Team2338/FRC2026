package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.Agitator.AgitatorAutoRun;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoRun;

public class CollectAutoSequence extends SequentialCommandGroup {
    /**
     * Not in use
     */
    public CollectAutoSequence() {
        addCommands(
                new ParallelCommandGroup(
                        new AgitatorAutoRun(),
                        new CollectorAutoRun(Constants.Collector.COLLECTOR_FAST_RPM)
                )
        );
    }

}
