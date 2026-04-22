package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.agitator.AgitatorEject;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.ReverseCollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPivotDown;
import team.gif.robot.commands.shooter.IndexerEject;
import team.gif.robot.commands.shooter.ShooterRPM;
import team.gif.robot.commands.shooter.shooterautos.IndexerAuton;

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
                new PrintCommand("THIS IS CRAZY!!!!!!"),
                new ReverseCollectorPercent(),
                new IndexerEject(),
                new AgitatorEject()
        );
    }
}
