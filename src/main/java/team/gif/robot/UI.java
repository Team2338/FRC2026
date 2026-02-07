package team.gif.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class UI {
    /**
     *  Widgets (e.g. gyro, text, True/False flags),
     *  buttons (e.g. SmartDashboard.putData("Reset", new ResetHeading()); ),
     *  and Chooser options (e.g. auto mode, auto delay)
     *
     *  Placed in SmartDashboard network table
     *  After dashboard loads for the first time, manually move items from network table onto respective dashboard tab
     *  and save file as "YYYY elastic-layout.json"
     */
    public UI() {
        SmartDashboard.putNumber("PID/P", 0.3);
        SmartDashboard.putNumber("PID/I", 0);
        SmartDashboard.putNumber("PID/D", 0);
        SmartDashboard.putNumber("PID/Percent", 0);
        SmartDashboard.putNumber("PID/Voltage", 0);
        SmartDashboard.putNumber("PID/Reference", 3500);
        SmartDashboard.putString("PID/Default Command", "Driver Left joystick - percent control");
        SmartDashboard.putString("PID/Percent BTN", "Driver Y");
        SmartDashboard.putString("PID/Voltage BTN", "Driver X");
        SmartDashboard.putString("PID/Reference BTN", "Driver A");
        SmartDashboard.putNumber("Indexer/Stage 1", 0.5);
        SmartDashboard.putNumber("Indexer/Stage 2", 0.8);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putNumber("Collector/PID/Collect P", 0);
        SmartDashboard.putNumber("Collector/PID/Collect I", 0);
        SmartDashboard.putNumber("Collector/PID/Collect D", 0);
//        SmartDashboard.putNumber("Collector/PID/Pivot P", 0);
//        SmartDashboard.putNumber("Collector/PID/Pivot I", 0);
//        SmartDashboard.putNumber("Collector/PID/Pivot D", 0);
        SmartDashboard.putNumber("Collector/PID/Collect Percent", 0);
        SmartDashboard.putNumber("Collector/PID/Collect Voltage", 0);
        SmartDashboard.putNumber("Collector/PID/Collect Reference", 0);
        SmartDashboard.putNumber("Collector/PID/Pivot Percent", 0);
        SmartDashboard.putNumber("Collector/PID/Pivot Voltage", 0);
    }

    /**
     * Widgets which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     SmartDashboard.putString("Elevator", String.format("%11.2f", Elevator.getPosition());
     */
    public void update() {
        //Example
        //SmartDashboard.putNumber("Climber Position", Robot.elevator.getPosition())
        SmartDashboard.putNumber("PID/Shooter 1 Speed", Robot.shooter.getSpeed());
        SmartDashboard.putNumber("PID/Shooter 2 Speed", Robot.shooter.getSpeed2());
        SmartDashboard.putNumber("PID/Shooter 3 Speed", Robot.shooter.getSpeed3());
        SmartDashboard.putNumber("PID/Shooter Current", Robot.shooter.getCurrent());
        SmartDashboard.putNumber("PID/Shooter Output", Robot.shooter.getOutput());
        SmartDashboard.putNumber("PID/Shooter Output 2", Robot.shooter.getOutput2());
        SmartDashboard.putNumber("PID/Shooter Output 3", Robot.shooter.getOutput3());
        SmartDashboard.putNumber("Indexer/Speed", Robot.indexer.getSpeed());
        SmartDashboard.putNumber("Collector/PID/Collector Output", Robot.collector.getCollectOutput());
        SmartDashboard.putNumber("Collector/PID/Collector Speed", Robot.collector.getCollectSpeed());
        SmartDashboard.putNumber("Collector/PID/Pivot Output", Robot.collector.getPivotOutput());
        SmartDashboard.putNumber("Collector/PID/Pivot Speed", Robot.collector.getPivotSpeed());

    }
}
