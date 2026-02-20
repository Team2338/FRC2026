package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.commands.Agitator.AgitatorAuto;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPercent;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPivotDown;

public class CollectAutoSequence extends SequentialCommandGroup {
    public CollectAutoSequence() {
        addCommands(
                new CollectorAutoPivotDown(),
                new ParallelRaceGroup(
                        new AgitatorAuto(),
                        new CollectorAutoPercent()
                )
        );
    }

}
