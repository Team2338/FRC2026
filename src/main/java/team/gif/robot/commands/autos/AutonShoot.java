package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.agitator.AgitatorPercent;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoAgitate;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPercent;
import team.gif.robot.commands.drivetrain.HubAutoAlign;
import team.gif.robot.commands.shooter.ShooterAuto;
import team.gif.robot.commands.shooter.shooterautos.IndexerAuton;
import team.gif.robot.commands.shooter.shooterautos.AutonShooterRPM;

public class AutonShoot extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos
     */
//    new ShooterAuto().alongWith(new HubAutoAlign()).alongWith(new CollectorPercent(0.9)).alongWith(new AgitatorPercent()
    public AutonShoot() {
        super(new AutonShooterRPM(Constants.Shooter.SHOOTER_AUTON_FAR_RPM).withTimeout(3.5));
        addCommands(
                new WaitCommand(0.25).andThen(new IndexerAuton(true)),
                new WaitCommand(0.75).andThen(new CollectorAutoAgitate()),
                new CollectorAutoPercent(Constants.Collector.COLLECTOR_FAST_PERCENT)
//                new ShooterAuto(),
//                new HubAutoAlign(),
//                new CollectorPercent(0.9),
//                new AgitatorPercent(),
//                new WaitCommand(0.75).andThen(new CollectorAutoAgitate())
        );
    }
}
