package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.Agitator.AgitatorAutonPercent;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoRPM;

public class CollectAutoSequence extends SequentialCommandGroup {
    /**
     * Not in use
     */
    public CollectAutoSequence() {
        addCommands(
                new ParallelCommandGroup(
                        new AgitatorAutonPercent(),
                        new CollectorAutoRPM(Constants.Collector.COLLECTOR_FAST_RPM)
                )
        );
    }

}
