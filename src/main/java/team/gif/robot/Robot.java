// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.subsystems.drivers.Pigeon2_0;
import team.gif.robot.subsystems.drivers.swerve.SwerveDrive;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConfiguration;
import team.gif.robot.subsystems.drivers.swerve.SwerveDrivetrain;
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

        swerveConfig = new SwerveConfiguration(new RobotMap.Mk5Map(), new Constants.Mk5Constants(), TalonFXDriveMotor::new, TalonFXTurnMotor::new, CANCoderEncoder::new);
//        swerveConfig = new SwerveConfiguration(new RobotMap.Mk4Map(), new Constants.Mk4Constants(), TalonFXDriveMotor::new, TalonFXTurnMotor::new, CANCoderEncoder::new);
//        swerveConfig = new SwerveConfiguration(new RobotMap.Mk3Map(), new Constants.Mk3Constants(), SparkMaxDriveMotor::new, TalonSRXTurnMotorEncoder::new, null);
        swerveDrive = new SwerveDrive(swerveConfig);
        swerveDrive.setDefaultCommand(new DriveSwerve());
        swerveDrive.enableDebugMode();
//        swerveDrive.addLimelight("limelight-front");

        //These should be at or near the bottom
        oi = new OI();
        ui = new UI();

        FollowPathCommand.warmupCommand().schedule();


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
    public void autonomousInit() {

        Command command1 = ui.fancyAutoChooser.getSelected();
        Command command2 = ui.fancyAutoChooser1.getSelected();
        Command command3 = ui.fancyAutoChooser2.getSelected();
        Command command4 = ui.fancyAutoChooser3.getSelected();
        Command command5 = ui.fancyAutoChooser4.getSelected();

        autonomousCommand = new SequentialCommandGroup(command1, command2, command3 , command4, command5);
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

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

    static public Command generateAuto(String pathName) {

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            //TODO: update constraints
            PathConstraints constraints = new PathConstraints(3,4, Units.degreesToRadians(540), Units.degreesToRadians(720));
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        } catch (Exception e) {
            DriverStation.reportError("Failed to generate Auto Path: " + e.getMessage(), e.getStackTrace());
            return Commands.none();

        }

    }
}
