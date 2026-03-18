package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPivotDown;
import team.gif.robot.commands.shooter.shooterautos.AutonShooterRPM;
import team.gif.robot.commands.shooter.shooterautos.IndexerAuton;

public class AutonCenterShoot extends ParallelDeadlineGroup {
    /**
     * This is the entire sequence to shoot in autos at the start of the match
     * with the collector up
     */
    public AutonCenterShoot() {
        super(new AutonShooterRPM(Constants.Shooter.SHOOTER_AUTON_FAR_RPM).withTimeout(5));
        addCommands(
                new CollectorAutoPivotDown(0.5),
                new WaitCommand(1.5).andThen(new IndexerAuton(false))
        );
    }
}
