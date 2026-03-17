package team.gif.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class UI {
    public SendableChooser<Double> delayChooser = new SendableChooser<>();
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
        SmartDashboard.putString("PID/Default Command", "Driver Left joystick - percent control");
        SmartDashboard.putString("PID/Percent BTN", "Driver Y");
        SmartDashboard.putString("PID/Voltage BTN", "Driver X");
        SmartDashboard.putString("PID/Reference BTN", "Driver A");
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
        SmartDashboard.putNumber("Auto Collector/PID/Collect Reference", 0);
        SmartDashboard.putNumber("Agitator/Agitator Percent", 0);

        delayChooser.setDefaultOption("0", 0.0);
        delayChooser.addOption("1", 1.0);
        delayChooser.addOption("2", 2.0);
        delayChooser.addOption("3", 3.0);
        delayChooser.addOption("4", 4.0);
        delayChooser.addOption("5", 5.0);
        delayChooser.addOption("6", 6.0);
        delayChooser.addOption("7", 7.0);
        delayChooser.addOption("8", 8.0);
        delayChooser.addOption("9", 9.0);
        delayChooser.addOption("10", 10.0);
        delayChooser.addOption("11", 11.0);
        delayChooser.addOption("12", 12.0);
        delayChooser.addOption("13", 13.0);
        delayChooser.addOption("14", 14.0);
        delayChooser.addOption("15", 15.0);
        delayChooser.addOption("16", 16.0);
        delayChooser.addOption("17", 17.0);
        delayChooser.addOption("18", 18.0);
        delayChooser.addOption("19", 19.0);
        delayChooser.addOption("20", 20.0);
        SmartDashboard.putData("Delay Chooser", delayChooser);
    }

    /**
     * Widgets which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     SmartDashboard.putString("Elevator", String.format("%11.2f", Elevator.getPosition());
     */
    public void update() {
        SmartDashboard.putNumber("PID/Shooter 1 Speed", Robot.shooter.getLeftMotorSpeed());
        SmartDashboard.putNumber("PID/Shooter 2 Speed", Robot.shooter.getMiddleMotorSpeed());
        SmartDashboard.putNumber("PID/Shooter 3 Speed", Robot.shooter.getRightMotorSpeed());
        SmartDashboard.putNumber("PID/Shooter Current", Robot.shooter.getLeftMotorCurrent());
        SmartDashboard.putNumber("PID/Shooter Output", Robot.shooter.getLeftMotorOutput());
        SmartDashboard.putNumber("PID/Shooter Output 2", Robot.shooter.getMiddleMotorOutput());
        SmartDashboard.putNumber("PID/Shooter Output 3", Robot.shooter.getRightMotorOutput());
        SmartDashboard.putNumber("Indexer/Speed", Robot.indexer.getBottomIndexerSpeed());
        SmartDashboard.putNumber("Collector/PID/Collector Output", Robot.collectMotor.getCollectOutput());
        SmartDashboard.putNumber("Collector/PID/Collector Speed", Robot.collectMotor.getCollectSpeed());
        SmartDashboard.putNumber("Collector/PID/Pivot Output", Robot.pivotMotor.getPivotOutput());
        SmartDashboard.putNumber("Collector/PID/Pivot Speed", Robot.pivotMotor.getPivotSpeed());
        SmartDashboard.putString("Collector/Collect Pos", String.format("%11.2f", Robot.pivotMotor.getPosition()));
        SmartDashboard.putString("Collector/Collect AbsPos", String.format("%11.4f", Robot.pivotMotor.getAbsEncoderPos()));
        SmartDashboard.putBoolean("Diagnostics/Motor Temp", Robot.diagnostics.anyMotorTempHot());
        SmartDashboard.putBoolean("Diagnostics/Swerve Motor Hot", Robot.diagnostics.swerveMotorTempHot());
        SmartDashboard.putBoolean("Diagnostics/Mechanism Motor Hot", Robot.diagnostics.mechanismMotorTempHot());
        SmartDashboard.putBoolean("Competition", Robot.isCompetition);
        SmartDashboard.putString("ShiftTime", String.format("%11.1f", Robot.shiftTime));
        SmartDashboard.putString("Shift",Robot.matchShift);
        updateTemps();
    }

    public void updateTemps(){
        if(!Robot.isCompetition) {
            SmartDashboard.putBoolean("MotorTemps/Agitator", Robot.agitator.isOverTemp());
            SmartDashboard.putBoolean("MotorTemps/Bottom Indexer", Robot.indexer.isBottomIndexerMotorOverTemp());
            SmartDashboard.putBoolean("MotorTemps/Top Indexer", Robot.indexer.isTopIndexerMotorOverTemp());
            SmartDashboard.putBoolean("MotorTemps/Left Shooter", Robot.shooter.isLeftMotorOverTemp());
            SmartDashboard.putBoolean("MotorTemps/Middle Shooter", Robot.shooter.isMiddleMotorOverTemp());
            SmartDashboard.putBoolean("MotorTemps/Right Shooter", Robot.shooter.isRightMotorOverTemp());
            SmartDashboard.putBoolean("MotorTemps/Pivot", Robot.pivotMotor.isOverTemp());
            SmartDashboard.putBoolean("MotorTemps/Collector", Robot.collectMotor.isOverTemp());
            SmartDashboard.putBoolean("MotorTemps/Swerve Module FR", Robot.swerveDrive.fR.isDriveMotorHot());
            SmartDashboard.putBoolean("MotorTemps/Swerve Module FL", Robot.swerveDrive.fL.isDriveMotorHot());
            SmartDashboard.putBoolean("MotorTemps/Swerve Module RL", Robot.swerveDrive.rL.isDriveMotorHot());
            SmartDashboard.putBoolean("MotorTemps/Swerve Module RR", Robot.swerveDrive.rR.isDriveMotorHot());
        }
    }
}
