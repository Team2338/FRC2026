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
        SmartDashboard.putNumber("PID/P", 0);
        SmartDashboard.putNumber("PID/I", 0);
        SmartDashboard.putNumber("PID/D", 0);
        SmartDashboard.putNumber("PID/Percent", 0);
        SmartDashboard.putNumber("PID/Voltage", 0);
        SmartDashboard.putNumber("PID/Reference", 0);
        SmartDashboard.putString("PID/Default Command", "Driver Left joystick - percent control");
        SmartDashboard.putString("PID/Percent BTN", "Driver A");
        SmartDashboard.putString("PID/Voltage BTN", "Driver B");
        SmartDashboard.putString("PID/Reference BTN", "Driver X");
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
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

//        SmartDashboard.putNumber("PID/Neo Speed", Robot.neo.getSpeed());
//        SmartDashboard.putNumber("PID/Neo Current", Robot.neo.getCurrent());

        SmartDashboard.putNumber("PID/Neo Speed", 0);
        SmartDashboard.putNumber("PID/Neo Current", 0);

    }
}
