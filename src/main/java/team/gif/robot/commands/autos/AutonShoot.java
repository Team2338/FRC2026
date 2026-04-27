package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.commands.agitator.AgitatorPercent;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoAgitate;
import team.gif.robot.commands.drivetrain.HubAutoAlign;
import team.gif.robot.commands.shooter.ShooterAuto;
import team.gif.robot.commands.shooter.shooterautos.RobotBeached;
import team.gif.robot.commands.shooter.shooterautos.RobotInNeutral;

public class AutonShoot extends ParallelCommandGroup {
    public static HubAutoAlign hubCommand = new HubAutoAlign();
    public static ShooterAuto shootCommand = new ShooterAuto();
    /**
     * This is the entire sequence to shoot, includes option to also agitate the collector
     *
     * @param auto True if sequence should be in auto mode (e.g. auto agitate, pose check), false if agitation is done externally (e.g. during teleop by aux controller)
     */
    public AutonShoot(boolean auto) {
        if (auto) {
            addCommands(
                    new ParallelRaceGroup(
                    new ShooterAuto(),
                    new HubAutoAlign(),
                    new CollectorPercent(0.8),
                    new AgitatorPercent(),
                    new WaitCommand(1.25).andThen(new CollectorAutoAgitate()),
                    new RobotInNeutral(),
                    new RobotBeached()
                    )
            );
        } else {
            addCommands(
                    shootCommand,
                    hubCommand,
                    new CollectorPercent(0.8),
                    new AgitatorPercent()
            );
        }
    }
}
