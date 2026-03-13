package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.lib.drivePace;
import team.gif.robot.commands.agitator.AgitatorPercent;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.ReverseCollectorPercent;
import team.gif.robot.commands.shooter.IndexerPercent;
import team.gif.robot.commands.shooter.IndexerReverse;
import team.gif.robot.commands.shooter.ShooterAuto;
import team.gif.robot.commands.shooter.ShooterRPM;
import team.gif.robot.commands.drivetrain.HubAutoAlign;

public class OI {
    /*
     * Instantiate all joysticks/controllers and their buttons here
     *
     * Examples:
     * public final CommandXboxController driver = new CommandXboxController(0);
     *
     * public final Trigger dA = driver.a();
     */

    public final CommandXboxController driver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER_ID);
    public final CommandXboxController aux = new CommandXboxController(RobotMap.AUX_CONTROLLER_ID);
    public final CommandXboxController test = new CommandXboxController(RobotMap.TEST_CONTROLLER_ID);

    public final Trigger dA = driver.a();
    public final Trigger dB = driver.b();
    public final Trigger dX = driver.x();
    public final Trigger dY = driver.y();
    public final Trigger dLBump = driver.leftBumper();
    public final Trigger dRBump = driver.rightBumper();
    public final Trigger dBack = driver.back();
    public final Trigger dStart = driver.start();
    public final Trigger dLStickBtn = driver.leftStick();
    public final Trigger dRStickBtn = driver.rightStick();
    public final Trigger dRTrigger = driver.rightTrigger();
    public final Trigger dLTrigger = driver.leftTrigger();
    public final Trigger dDPadUp = driver.povUp();
    public final Trigger dDPadRight = driver.povRight();
    public final Trigger dDPadDown = driver.povDown();
    public final Trigger dDPadLeft = driver.povLeft();

    public final Trigger aA = aux.a();
    public final Trigger aB = aux.b();
    public final Trigger aX = aux.x();
    public final Trigger aY = aux.y();
    public final Trigger aLBump = aux.leftBumper();
    public final Trigger aRBump = aux.rightBumper();
    public final Trigger aBack = aux.back();
    public final Trigger aStart = aux.start();
    public final Trigger aLStickBtn = aux.leftStick();
    public final Trigger aRStickBtn = aux.rightStick();
    public final Trigger aRTrigger = aux.rightTrigger();
    public final Trigger aLTrigger = aux.leftTrigger();
    public final Trigger aDPadUp = aux.povUp();
    public final Trigger aDPadRight = aux.povRight();
    public final Trigger aDPadDown = aux.povDown();
    public final Trigger aDPadLeft = aux.povLeft();

    public final Trigger tA = test.a();
    public final Trigger tB = test.b();
    public final Trigger tX = test.x();
    public final Trigger tY = test.y();
    public final Trigger tLBump = test.leftBumper();
    public final Trigger tRBump = test.rightBumper();
    public final Trigger tBack = test.back();
    public final Trigger tStart = test.start();
    public final Trigger tLStickBtn = test.leftStick();
    public final Trigger tRStickBtn = test.rightStick();
    public final Trigger tRTrigger = test.rightTrigger();
    public final Trigger tLTrigger = test.leftTrigger();
    public final Trigger tDPadUp = test.povUp();
    public final Trigger tDPadRight = test.povRight();
    public final Trigger tDPadDown = test.povDown();
    public final Trigger tDPadLeft = test.povLeft();

    public OI() {
        DriverStation.silenceJoystickConnectionWarning(true);

        /*
         *
         * Create controller actions here
         *
         * Usages:
         * dRTrigger.whileTrue(new CollectCommand());
         * dLTrigger.onTrue(new EjectCommand());
         * dA.whileTrue(new RepeatCommand(new RapidFire());
         * aStart.onTrue(new InstantCommand(Robot.elevator::zeroEncoder).ignoringDisable(true));
         *
         * onTrue (fka whenPressed)    Init->Execute repeats until IsFinished = true->End, will not start again at Init if still held down
         * whileTrue (fka whenHeld)    Init->Execute repeats until IsFinished = true or button released->End, will not start again at Init if still held down
         * whileTrue(new RepeatCommand()) (fka whileHeld)   Init->Execute repeats until IsFinished = true or button released->End, will start again at Init if still held down
         *
         * Simple Test:
         *   aX.onTrue(new PrintCommand("aX"));
         */
        dStart.and(dDPadUp).onTrue(new InstantCommand(Robot.pigeon::resetPigeonPosition).ignoringDisable(true));
        dStart.and(dDPadDown).onTrue(new InstantCommand(() -> Robot.pigeon.resetPigeonPosition(180)).ignoringDisable(true));
        dStart.and(dDPadRight).onTrue(new InstantCommand(Robot.pivotMotor::zeroEncoder).ignoringDisable(true));
        dStart.and(dDPadLeft).onTrue(new InstantCommand(Robot.pivotMotor::deployedEncoder).ignoringDisable(true));

        dLBump.whileTrue(new IndexerReverse(0.75, 0.25).andThen(new IndexerPercent(0.8, 0.5))); //Main index button for shooting - back then forward
        dRBump.onTrue(new InstantCommand(() ->  Robot.swerveDrive.setDrivePace(drivePace.BOOST_FR)));
        dRBump.onFalse(new InstantCommand(() ->  Robot.swerveDrive.setDrivePace(drivePace.COAST_FR)));
        dLTrigger.whileTrue(new ShooterRPM(4000)); // Long shot to use  if cameras break // TODO change value later
        dRTrigger.whileTrue(new ShooterRPM(3000)); // Short shot to use if cameras break // TODO change value later

        dA.whileTrue(new HubAutoAlign());
        dB.whileTrue(new ReverseCollectorPercent(1.0).alongWith(new IndexerReverse(0.75)).alongWith(new AgitatorPercent(-0.3))); //Fuel eject from collector
        dX.whileTrue(new ShooterAuto());
        dY.whileTrue(new ShooterRPM(4500)); //High RPM to pass/shuttle fuel from neutral zone to alliance
//        dY.onTrue(new AutonShoot());
        //dLBump.onTrue(new WaitCommand(0.4).andThen(new CollectorAutoPivot().withTimeout(1.3)));
        //dBack.and(dX).onTrue(Robot.auto);

        aStart.and(aDPadUp).onTrue(new InstantCommand(Robot.pigeon::resetPigeonPosition).ignoringDisable(true));
        aStart.and(aDPadDown).onTrue(new InstantCommand(() -> Robot.pigeon.resetPigeonPosition(180)).ignoringDisable(true));
        aStart.and(aDPadRight).onTrue(new InstantCommand(Robot.pivotMotor::zeroEncoder).ignoringDisable(true));
        aStart.and(aDPadLeft).onTrue(new InstantCommand(Robot.pivotMotor::deployedEncoder).ignoringDisable(true));

        aLTrigger.whileTrue(new CollectorPercent(0.5).alongWith(new AgitatorPercent())); //Slow Collect
        aRTrigger.whileTrue(new CollectorPercent(0.9).alongWith(new AgitatorPercent())); //Fast Collect

        aRBump.whileTrue(new ReverseCollectorPercent(0.25));
    }

    public void setRumble(boolean doRumble){
        driver.setRumble(GenericHID.RumbleType.kBothRumble, doRumble ? 1.0 : 0.0);
        aux.setRumble(GenericHID.RumbleType.kBothRumble, doRumble ? 1.0 : 0.0);
    }
}
