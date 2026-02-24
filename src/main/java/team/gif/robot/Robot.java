// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import team.gif.robot.commands.Collector.CollectorPercent;
import team.gif.robot.commands.Collector.CollectorPivot;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.subsystems.Agitator;
import team.gif.robot.subsystems.Collector.CollectMotor;
import team.gif.robot.subsystems.Collector.PivotMotor;
import team.gif.robot.subsystems.Indexer;
import team.gif.robot.subsystems.Shooter;
import team.gif.robot.subsystems.drivers.Pigeon2_0;
import team.gif.robot.subsystems.drivers.swerve.SwerveDrive;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConfiguration;
import team.gif.robot.subsystems.drivers.swerve.TalonFXDriveMotor;
import team.gif.robot.subsystems.drivers.swerve.TalonFXTurnMotor;
import team.gif.robot.subsystems.drivers.swerve.CANCoderEncoder;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static Command autonomousCommand;
    private RobotContainer robotContainer;
    public static OI oi;

    public static Pigeon2_0 pigeon;

    public static SwerveConfiguration swerveConfig;
    public static SwerveDrive swerveDrive;

    public static UI ui;

    public static Shooter shooter;
    public static Indexer indexer;

    public static CollectMotor collectMotor;
    public static PivotMotor pivotMotor;

    public static Agitator agitator;

    public static final boolean enableSwerveDebug = true;
    public static final boolean fullDashboard = true;


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        pigeon = new Pigeon2_0(RobotMap.PIGEON_ID);

        shooter = new Shooter();
        indexer = new Indexer();

        collectMotor = new CollectMotor();
        pivotMotor = new PivotMotor();
        pivotMotor.setDefaultCommand(new CollectorPivot());

        agitator = new Agitator();

        swerveConfig = new SwerveConfiguration(new RobotMap.Mk5Map(), new Constants.Mk5Constants(), TalonFXDriveMotor::new, TalonFXTurnMotor::new, CANCoderEncoder::new);
        swerveDrive = new SwerveDrive(swerveConfig);
        swerveDrive.setDefaultCommand(new DriveSwerve());
        swerveDrive.enableDebugMode();
//        swerveDrive.addPhotonCamera("left-cam", Constants.Vision.LEFT_CAMERA_POSITION);
        swerveDrive.addPhotonCamera("right-cam", Constants.Vision.RIGHT_CAMERA_POSITION);
//        swerveDrive.addPhotonCamera("photonvision-side", Constants.Vision.SIDE_CAMERA_POSITION);

        //These should be at or near the bottom
        oi = new OI();
        ui = new UI();
        pigeon.addToShuffleboard("Heading");


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
        CommandScheduler.getInstance().run();

        ui.update();

    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
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
}
