package team.gif.robot.commands.SequentialAutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import team.gif.robot.Constants;
import team.gif.robot.commands.Agitator.AgitatorAuto;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAuton;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRPM;

public class InitialAutonShoot extends ParallelCommandGroup {

    public InitialAutonShoot() {
        addCommands(
                new ShooterAutoRPM().withTimeout(Constants.Shooter.SHOOTER_AUTO_SECONDS),
                new AgitatorAuto(),
                new IndexerAuton()
        );
    }
}
