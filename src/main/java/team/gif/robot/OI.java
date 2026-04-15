package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.lib.drivePace;
import team.gif.robot.commands.agitator.AgitatorEject;
import team.gif.robot.commands.agitator.AgitatorPercent;
import team.gif.robot.commands.autos.AutonShoot;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.CollectorTeleopPivotDown;
import team.gif.robot.commands.collector.ReverseCollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAgitateOnce;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoAgitate;
import team.gif.robot.commands.shooter.IndexerEject;
import team.gif.robot.commands.shooter.PassAuto;
import team.gif.robot.commands.shooter.ShooterDashboard;
import team.gif.robot.commands.shooter.ShooterRPM;
import team.gif.robot.commands.shooter.ToggleShooterManualMode;

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

    public final Trigger manualMode = new Trigger(AutonShoot.shootCommand::getManualMode);

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

        //Driver Controls
        dLStickBtn.and(dRStickBtn).and(dRTrigger.negate()).whileTrue(new RepeatCommand(new InstantCommand(Robot.swerveDrive::xStance, Robot.swerveDrive))); //Swerve X Stance to not get pushed around;
        dLStickBtn.and(dRStickBtn).and(dRTrigger).onTrue(new InstantCommand(() -> AutonShoot.hubCommand.setXStance(true))); //Swerve X Stance to not get pushed around;
        dLStickBtn.and(dRStickBtn).and(dRTrigger).onFalse(new InstantCommand(() -> AutonShoot.hubCommand.setXStance(false))); //Swerve X Stance to not get pushed around;

        dStart.and(dDPadUp).onTrue(new InstantCommand(Robot.pigeon::resetPigeonPosition).ignoringDisable(true));
        dStart.and(dDPadDown).onTrue(new InstantCommand(() -> Robot.pigeon.resetPigeonPosition(180)).ignoringDisable(true));
        dStart.and(dDPadRight).onTrue(new InstantCommand(() -> Robot.pigeon.resetPigeonPosition(90)).ignoringDisable(true));
        dStart.and(dDPadLeft).onTrue(new InstantCommand(() -> Robot.pigeon.resetPigeonPosition(270)).ignoringDisable(true));
//        dStart.and(dDPadRight).onTrue(new InstantCommand(Robot.pivotMotor::zeroEncoder).ignoringDisable(true));
//        dStart.and(dDPadLeft).onTrue(new InstantCommand(Robot.pivotMotor::deployedEncoder).ignoringDisable(true));

        dRBump.and(dLBump.negate()).onTrue(new InstantCommand(() ->  Robot.swerveDrive.setDrivePace(drivePace.BOOST_FR)));
        dRBump.and(dLBump.negate()).onFalse(new InstantCommand(() ->  Robot.swerveDrive.setDrivePace(drivePace.COAST_FR)));

        dLBump.and(dRBump.negate()).whileTrue(new ShooterRPM(3500).alongWith(new AgitatorPercent()).alongWith( new CollectorPercent(0.8)).alongWith(Constants.Indexer.getFullCommand())); // Passing fuel from neutral zone to our alliance zone
        dLBump.and(dRBump.negate()).onFalse(new ShooterRPM(3500).withTimeout(0.5));
        dLBump.and(dRBump).whileTrue(new ReverseCollectorPercent(1.0).alongWith(new IndexerEject()).alongWith(new AgitatorEject())); //Fuel eject from collector


        dRTrigger.and(manualMode).whileTrue(new ShooterRPM(3325).alongWith(new AgitatorPercent()).alongWith( new CollectorPercent(0.8)).alongWith(Constants.Indexer.getFullCommand()));
        dRTrigger.and(manualMode).onFalse(new ShooterRPM(3325).withTimeout(0.25)); //Keep shooter running after command
        dRTrigger.and(manualMode.negate()).whileTrue(new AutonShoot(false).alongWith(new InstantCommand(() -> AutonShoot.hubCommand.setXStance(true)))); //Full sequence to align, rev shooter, and index (also manual long shot if cameras break)
//        dRTrigger.and(manualMode.negate()).whileTrue(new AutonShoot(false)); //Full sequence to align, rev shooter, and index (also manual long shot if cameras break)
        dRTrigger.and(manualMode.negate()).onFalse(new ShooterRPM(3500).withTimeout(0.25)); //Full sequence to align, rev shooter, and index (also manual long shot if cameras break)

        dLTrigger.whileTrue(new ShooterRPM(3130).alongWith(new AgitatorPercent()).alongWith( new CollectorPercent(0.8)).alongWith(Constants.Indexer.getFullCommand())); // Short shot to use if cameras break
        dLTrigger.onFalse(new ShooterRPM(3130).withTimeout(0.25));

//        dDPadUp.and(dStart.negate()).whileTrue(new ShooterRPM(3500).alongWith(new AgitatorPercent()).alongWith( new CollectorPercent(0.9)).alongWith(ShooterAuto.indexFullCommand)); // Long shot to use  if cameras break
//        dDPadDown.and(dStart.negate()).whileTrue(new ShooterRPM(3300).alongWith(new AgitatorPercent()).alongWith( new CollectorPercent(0.9)).alongWith(ShooterAuto.indexFullCommand)); // Short shot to use if cameras break
//        dDPadRight.and(dStart.negate()).whileTrue(new ShooterRPM(4000).alongWith(new AgitatorPercent()).alongWith( new CollectorPercent(0.9)).alongWith(ShooterAuto.indexFullCommand)); // Passing fuel from neutral zone to our alliance zone

        dBack.and(dX).onTrue(new InstantCommand(Robot::runAutonomousCommand));
        dBack.and(dB).onTrue(new InstantCommand(Robot::cancelAutonomousCommand));

        // for driver only controls
        dX.and(dBack.negate()).whileTrue(new CollectorPercent(1.0).alongWith(new AgitatorPercent())); //Standard Collect
        dB.and(dBack.negate()).onTrue(new CollectorAutoAgitate()); //Standard agitator sequence for shooting
        // For shot calibration
        dA.whileTrue(new ShooterDashboard().alongWith(new AgitatorPercent()).alongWith( new CollectorPercent(0.8)).alongWith(Constants.Indexer.getFullCommand()));

        //Aux Controls
        aStart.and(aDPadUp).onTrue(new InstantCommand(Robot.pigeon::resetPigeonPosition).ignoringDisable(true));
        aStart.and(aDPadDown).onTrue(new InstantCommand(() -> Robot.pigeon.resetPigeonPosition(180)).ignoringDisable(true));
        aStart.and(aDPadRight).onTrue(new InstantCommand(() -> Robot.pigeon.resetPigeonPosition(90)).ignoringDisable(true));
        aStart.and(aDPadLeft).onTrue(new InstantCommand(() -> Robot.pigeon.resetPigeonPosition(270)).ignoringDisable(true));
//        aStart.and(aDPadRight).onTrue(new InstantCommand(Robot.pivotMotor::zeroEncoder).ignoringDisable(true));
//        aStart.and(aDPadLeft).onTrue(new InstantCommand(Robot.pivotMotor::deployedEncoder).ignoringDisable(true));

        aY.onTrue(new InstantCommand(() -> {
            AutonShoot.hubCommand.resetRotationOffset();
            AutonShoot.shootCommand.resetRPMOffset();
        }).ignoringDisable(true));
        aRBump.whileTrue(new ReverseCollectorPercent(0.25)); //Runs collector (only) in reverse to eject jammed fuel

        aLBump.and(manualMode).whileTrue(new ShooterRPM(3500).alongWith(new AgitatorPercent()).alongWith( new ReverseCollectorPercent(1.0)).alongWith(Constants.Indexer.getFullCommand())); // Passing fuel from neutral zone to our alliance zone
        aLBump.and(manualMode.negate()).whileTrue(new PassAuto());

        aLTrigger.whileTrue(new ReverseCollectorPercent(1.0).alongWith(new IndexerEject()).alongWith(new AgitatorEject())); //Standard fuel eject from collector
        aRTrigger.whileTrue(new CollectorPercent(1.0).alongWith(new AgitatorPercent())); //Standard Collect (Runs Collector and Agitator (into indexer))

        aX.onTrue(new CollectorTeleopPivotDown()); //Rotates collector out
        aA.onTrue(new CollectorAgitateOnce()); //Lifts collector only once (for agitation)
        aB.onTrue(new CollectorAutoAgitate()); //Standard agitator sequence for shooting

        aDPadUp.and(aStart.negate()).onTrue(new InstantCommand(() -> AutonShoot.shootCommand.adjustRPMOffset(25)).ignoringDisable(true));
        aDPadLeft.and(aStart.negate()).onTrue(new InstantCommand(() -> AutonShoot.hubCommand.adjustRotationOffset(1)).ignoringDisable(true));
        aDPadRight.and(aStart.negate()).onTrue(new InstantCommand(() -> AutonShoot.hubCommand.adjustRotationOffset(-1)).ignoringDisable(true));
        aDPadDown.and(aStart.negate()).onTrue(new InstantCommand(() -> AutonShoot.shootCommand.adjustRPMOffset(-25)).ignoringDisable(true));

        aStart.and(aBack).toggleOnTrue(new ToggleShooterManualMode().ignoringDisable(true));
    }

    public void setRumble(boolean doRumble){
        driver.setRumble(GenericHID.RumbleType.kBothRumble, doRumble ? 1.0 : 0.0);
        aux.setRumble(GenericHID.RumbleType.kBothRumble, doRumble ? 1.0 : 0.0);
    }
}
