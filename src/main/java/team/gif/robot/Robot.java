// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.robot.commands.collector.CollectorPivot;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.subsystems.Agitator;
import team.gif.robot.subsystems.collector.CollectMotor;
import team.gif.robot.subsystems.collector.PivotMotor;
import team.gif.robot.subsystems.Indexer;
import team.gif.robot.subsystems.Shooter;
import team.gif.robot.subsystems.drivers.Pigeon2_0;
import team.gif.robot.subsystems.drivers.swerve.SwerveDrive;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConfiguration;
import team.gif.robot.subsystems.drivers.swerve.TalonFXDriveMotor;
import team.gif.robot.subsystems.drivers.swerve.TalonFXTurnMotor;
import team.gif.robot.subsystems.drivers.swerve.CANCoderEncoder;

import static team.gif.robot.Constants.MatchTimes;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static Command autonomousCommand;
    private static RobotContainer robotContainer;
    public static OI oi;

    public static Pigeon2_0 pigeon;

    public static SwerveConfiguration swerveConfig;
    public static SwerveDrive swerveDrive;

    public static UI ui;
    public static Diagnostics diagnostics;

    public static Shooter shooter;
    public static Indexer indexer;
    public static Agitator agitator;

    public static CollectMotor collectMotor;
    public static PivotMotor pivotMotor;

    private double delay;
    private final Timer delayTimer = new Timer();
    public static boolean isCompetition = true;

    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    public static final boolean enableSwerveDebug = true;
    public static final boolean fullDashboard = true;

    public static String matchShift = "N/A";
    public static double shiftTime = 0;
    public static  boolean winAutoShiftTwoShiftFour;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        pigeon = new Pigeon2_0(RobotMap.PIGEON_ID);

        shooter = new Shooter();
        indexer = new Indexer();
        agitator = new Agitator();

        collectMotor = new CollectMotor();
        pivotMotor = new PivotMotor();

        swerveConfig = new SwerveConfiguration(new RobotMap.Mk5Map(), new Constants.Mk5Constants(), TalonFXDriveMotor::new, TalonFXTurnMotor::new, CANCoderEncoder::new);
        swerveDrive = new SwerveDrive(swerveConfig);
        swerveDrive.enableDebugMode();
        swerveDrive.addPhotonCamera("left-cam", Constants.Vision.LEFT_CAMERA_POSITION);
        swerveDrive.addPhotonCamera("right-cam", Constants.Vision.RIGHT_CAMERA_POSITION);
        swerveDrive.addPhotonCamera("side-cam", Constants.Vision.SIDE_CAMERA_POSITION);
        robotContainer = new RobotContainer();

        swerveDrive.setDefaultCommand(new DriveSwerve());
        //These should be at or near the bottom
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        oi = new OI();
        diagnostics = new Diagnostics();
        ui = new UI();
        pigeon.addToShuffleboard("Heading");

        winAutoShiftTwoShiftFour = false;
//        SignalLogger.start();

        PortForwarder.add(5800, "10.23.38.50", 5800);
        PortForwarder.add(1182, "10.23.38.50", 1182);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        commandScheduler.run();

        ui.update();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
//        commandScheduler.schedule(new WaitCommand(8).andThen(new InstantCommand(() -> SignalLogger.stop())));
        oi.setRumble(false);
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        delay = ui.delayChooser.getSelected();
        if(delay == 0.0 && autonomousCommand != null) {
            commandScheduler.schedule(autonomousCommand);
        } else if(delay != 0.0 && autonomousCommand != null) {
            delayTimer.reset();
            delayTimer.start();
        }

        matchShift = "Go Rock!";
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        if(delay > delayTimer.get() && autonomousCommand != null) {
            commandScheduler.schedule(autonomousCommand);
        }
        delayTimer.stop();

        if (!isCompetition && diagnostics.anyMotorTempHot()) {
            autonomousCommand.cancel();
            System.out.println(Timer.getFPGATimestamp() + " Driving has been disabled. There is a motor which exceeds the safe temperature threshold.");
        }
        shiftTime = DriverStation.getMatchTime();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        pivotMotor.setDefaultCommand(new CollectorPivot());
        winAutoShiftTwoShiftFour = false;
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double matchTime = DriverStation.getMatchTime();

        updateMatchInfo(matchTime);

        //Set the controllers to rumble throughout different period of the match.
        oi.setRumble(
                isBetweenTime(matchTime, MatchTimes.END_OF_TRANSITION_PERIOD + 5.0, 5.0) || //Rumble for 3 seconds before the transition period ends

                isBetweenTime(matchTime, MatchTimes.END_OF_FIRST_SHIFT +  10.0, 0.3) || //Double quick rumble 10 seconds before the first shift ends
                isBetweenTime(matchTime, MatchTimes.END_OF_FIRST_SHIFT +   9.6, 0.3) ||

                isBetweenTime(matchTime, MatchTimes.END_OF_FIRST_SHIFT +   5.0, 5.0) || //Rumble for 3 seconds before the first shift ends

                isBetweenTime(matchTime, MatchTimes.END_OF_SECOND_SHIFT + 10.0, 0.3) || //Double quick rumble 10 seconds before the second shift ends
                isBetweenTime(matchTime, MatchTimes.END_OF_SECOND_SHIFT +  9.6, 0.3) ||

                isBetweenTime(matchTime, MatchTimes.END_OF_SECOND_SHIFT +  5.0, 5.0) || //Rumble for 3 seconds before the second shift ends.

                isBetweenTime(matchTime, MatchTimes.END_OF_THIRD_SHIFT +  10.0, 0.3) || //Double quick rumble 10 seconds before the third shift ends
                isBetweenTime(matchTime, MatchTimes.END_OF_THIRD_SHIFT +   9.6, 0.3) ||

                isBetweenTime(matchTime, MatchTimes.END_OF_THIRD_SHIFT +   5.0, 5.0) || //Rumble for 3 seconds before the third shift ends

                isBetweenTime(matchTime, MatchTimes.END_OF_FOURTH_SHIFT + 10.0, 0.3) || //Double quick rumble 10 seconds before the fourth shift ends
                isBetweenTime(matchTime, MatchTimes.END_OF_FOURTH_SHIFT +  9.6, 0.3) ||

                isBetweenTime(matchTime, MatchTimes.END_OF_FOURTH_SHIFT +  5.0, 5.0) || //Rumble for 3 seconds before the fourth shift ends

                isBetweenTime(matchTime, MatchTimes.END_OF_MATCH + 10.0, 0.3) || //Double quick rumble 10 seconds before the match ends
                isBetweenTime(matchTime, MatchTimes.END_OF_MATCH +  9.6, 0.3) ||

                isBetweenTime(matchTime, MatchTimes.END_OF_MATCH +  5.0, 4.0) //Rumble for 2 seconds 3 seconds before the match ends
        );
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        commandScheduler.cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    public static void runAutonomousCommand() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }
    public static void cancelAutonomousCommand() {
        autonomousCommand.cancel();
    }

    /**
     * Checks if the current match time is in the specified interval.
     * @param matchTime Time for the interval to be analyzed against
     * @param startTime The beginning of the time interval to be analyzed.
     * @param duration The duration of the time interval to be analyzed.
     * @return True if the match time is between the interval, false if not.
     */
    public boolean isBetweenTime(double matchTime, double startTime, double duration) {
        double endTime = startTime - duration;
        return startTime > matchTime && matchTime > endTime;
    }

    /**
     * Updates the game data for match shift, remaining shift time, determines if
     * alliance should be collecting/passing or scoring
     * @param matchTime current match time from driver station
     */
    private void updateMatchInfo(double matchTime) {
        String gameData = DriverStation.getGameSpecificMessage();

        // if gamedata matches alliance color then we won auto and score in shifts 2 and 4
        // need to make sure the data is there or it will crash
        if (gameData != null && !gameData.isEmpty() && DriverStation.getAlliance().isPresent()) {
            if (gameData.charAt(0) == (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 'R' : 'B')) {
                winAutoShiftTwoShiftFour = true;
            }
        }

        // calculate which shift is active, shoot or passing, and remaining time in current shift
        if (matchTime > MatchTimes.END_OF_TRANSITION_PERIOD) {
            matchShift = winAutoShiftTwoShiftFour ? "Trans->Pass" : "Trans->Shoot";
            shiftTime = matchTime - MatchTimes.END_OF_TRANSITION_PERIOD;
        } else if (matchTime < MatchTimes.END_OF_FOURTH_SHIFT) {
            matchShift = "End Game";
            shiftTime = matchTime;
        } else {
            matchShift = "N/A";
            if (matchTime > MatchTimes.END_OF_FIRST_SHIFT) {
                matchShift = (winAutoShiftTwoShiftFour ? "1 Pass" : "1 Shoot");
            } else if (matchTime > MatchTimes.END_OF_SECOND_SHIFT) {
                matchShift = (winAutoShiftTwoShiftFour ? "2 Shoot" : "2 Pass");
            } else if (matchTime > MatchTimes.END_OF_THIRD_SHIFT) {
                matchShift = (winAutoShiftTwoShiftFour ? "3 Pass" : "3 Shoot");
            } else if (matchTime > MatchTimes.END_OF_FOURTH_SHIFT) {
                matchShift = (winAutoShiftTwoShiftFour ? "4 Shoot" : "4 Pass");
            }

            shiftTime = (matchTime>MatchTimes.END_OF_FIRST_SHIFT) ? (matchTime - MatchTimes.END_OF_FIRST_SHIFT) :
                        (matchTime>MatchTimes.END_OF_SECOND_SHIFT) ? (matchTime - MatchTimes.END_OF_SECOND_SHIFT) :
                        (matchTime>MatchTimes.END_OF_THIRD_SHIFT) ? (matchTime - MatchTimes.END_OF_THIRD_SHIFT) :
                        matchTime - MatchTimes.END_OF_FOURTH_SHIFT;
        }
    }
}
