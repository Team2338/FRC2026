package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.commands.agitator.AgitatorPercent;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoAgitate;
import team.gif.robot.commands.drivetrain.HubAutoAlign;
import team.gif.robot.commands.shooter.ShooterAuto;

public class AutonShoot extends ParallelCommandGroup {
    /**
     * This is the entire sequence to shoot in autos
     */
    public AutonShoot(boolean agitate) {
        if (agitate) {
            addCommands(
                    new ShooterAuto(),
                    new HubAutoAlign(),
                    new CollectorPercent(0.9),
                    new AgitatorPercent(),
                    new WaitCommand(0.75).andThen(new CollectorAutoAgitate())
            );
        } else {
            addCommands(
                    new ShooterAuto(),
                    new HubAutoAlign(),
                    new CollectorPercent(0.9),
                    new AgitatorPercent()
            );
        }
    }
}
