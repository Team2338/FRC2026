package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPivotDown;


public class AutonCollectDown extends SequentialCommandGroup {

    public AutonCollectDown() {
        addCommands(
                new CollectorAutoPivotDown(0.2).withTimeout(1.5).andThen(new CollectorAutoPivotDown(0.1))
        );
    }
}
