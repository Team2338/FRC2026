package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.commands.Agitator.AgitatorAuto;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPivotDown;
import team.gif.robot.commands.Collector.CollectorPercent;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAuton;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRPM;

public class InitialAutonShoot extends ParallelDeadlineGroup {

    public InitialAutonShoot() {
        super(new ShooterAutoRPM().withTimeout(Constants.Shooter.SHOOTER_AUTO_SECONDS));
        addCommands(
//                new AgitatorAuto(),
                new CollectorAutoPivotDown().withTimeout(1.5),
                new WaitCommand(1.5).andThen(new IndexerAuton())
//                new IndexerAuton()
        );
    }
}
