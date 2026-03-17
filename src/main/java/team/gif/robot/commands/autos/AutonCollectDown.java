package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPivotDown;


public class AutonCollectDown extends SequentialCommandGroup {
    /**
     * Pivots collector down and then holds collector down at a small power
     */
    public AutonCollectDown() {
        addCommands(
                new CollectorAutoPivotDown().withTimeout(1.5).andThen(new CollectorAutoPivotDown())
        );
    }
}
