package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.commands.agitator.AgitatorEject;
import team.gif.robot.commands.collector.ReverseCollectorPercent;
import team.gif.robot.commands.shooter.IndexerEject;

public class AutonEject extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos at the start of the match
     * from the Center shot. This sequence deploys the collector only part way in
     * order to keep the fuel close to the shooter. Also runs the collector to
     * move the fuel initially stored in the collector to the shooter. Does not run agitator
     * wheels because there are only 8 fuel in the hopper and no need to run the agitator.
     */
    public AutonEject() {
        super(new WaitCommand(3.0)); // max amount of time to run
        addCommands(
                new ReverseCollectorPercent(1.0),
                new IndexerEject(),
                new AgitatorEject()
        );
    }
}
