package team.gif.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team.gif.robot.commands.Shoot;

public class UI {
    public SendableChooser<Command> fancyAutoChooser = new SendableChooser<>();
    public SendableChooser<Command> fancyAutoChooser1 = new SendableChooser<>();
    public SendableChooser<Command> fancyAutoChooser2 = new SendableChooser<>();
    public SendableChooser<Command> fancyAutoChooser3 = new SendableChooser<>();
    public SendableChooser<Command> fancyAutoChooser4 = new SendableChooser<>();

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
        fancyAutoChooser.setDefaultOption("None", Commands.none());
        fancyAutoChooser.addOption("Shoot", new Shoot());
        fancyAutoChooser.addOption("Path1", Robot.generateAuto("path1"));
        fancyAutoChooser.addOption("Path2", Robot.generateAuto("path2"));
        fancyAutoChooser.addOption("Path3", Robot.generateAuto("path3"));
        SmartDashboard.putData("Fancy Auto Chooser", fancyAutoChooser);

        fancyAutoChooser1.setDefaultOption("None", Commands.none());
        fancyAutoChooser1.addOption("Shoot", new Shoot());
        fancyAutoChooser1.addOption("Path1", Robot.generateAuto("path1"));
        fancyAutoChooser1.addOption("Path2", Robot.generateAuto("path2"));
        fancyAutoChooser1.addOption("Path3", Robot.generateAuto("path3"));
        SmartDashboard.putData("Fancy Auto Chooser 1", fancyAutoChooser);

        fancyAutoChooser2.setDefaultOption("None", Commands.none());
        fancyAutoChooser2.addOption("Shoot", new Shoot());
        fancyAutoChooser2.addOption("Path1", Robot.generateAuto("path1"));
        fancyAutoChooser2.addOption("Path2", Robot.generateAuto("path2"));
        fancyAutoChooser2.addOption("Path3", Robot.generateAuto("path3"));
        SmartDashboard.putData("Fancy Auto Chooser 2", fancyAutoChooser);

        fancyAutoChooser3.setDefaultOption("None", Commands.none());
        fancyAutoChooser3.addOption("Shoot", new Shoot());
        fancyAutoChooser3.addOption("Path1", Robot.generateAuto("path1"));
        fancyAutoChooser3.addOption("Path2", Robot.generateAuto("path2"));
        fancyAutoChooser3.addOption("Path3", Robot.generateAuto("path3"));
        SmartDashboard.putData("Fancy Auto Chooser 3", fancyAutoChooser);

        fancyAutoChooser4.setDefaultOption("None", Commands.none());
        fancyAutoChooser4.addOption("Shoot", new Shoot());
        fancyAutoChooser4.addOption("Path1", Robot.generateAuto("path1"));
        fancyAutoChooser4.addOption("Path2", Robot.generateAuto("path2"));
        fancyAutoChooser4.addOption("Path3", Robot.generateAuto("path3"));
        SmartDashboard.putData("Fancy Auto Chooser 4", fancyAutoChooser);
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
    }
}
