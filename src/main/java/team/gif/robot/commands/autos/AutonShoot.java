package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.commands.agitator.AgitatorPercent;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoAgitate;
import team.gif.robot.commands.drivetrain.HubAutoAlign;
import team.gif.robot.commands.shooter.ShooterAuto;

public class AutonShoot extends ParallelCommandGroup {
    public static HubAutoAlign hubCommand = new HubAutoAlign();
    public static ShooterAuto shootCommand = new ShooterAuto();
    /**
     * This is the entire sequence to shoot, includes option to also agitate the collector
     *
     * @param agitate True if sequence should also agitate, false if agitation is done by aux controller
     */
    public AutonShoot(boolean agitate) {
        if (agitate) {
            addCommands(
                    new ShooterAuto(),
                    new HubAutoAlign(),
                    new CollectorPercent(0.9),
                    new AgitatorPercent(),
                    new WaitCommand(0.85).andThen(new CollectorAutoAgitate())
            );
        } else {
            addCommands(
                    shootCommand,
                    hubCommand,
                    new CollectorPercent(0.9),
                    new AgitatorPercent()
            );
        }
    }
}
