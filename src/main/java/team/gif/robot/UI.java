package team.gif.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.delay;
import team.gif.robot.commands.autos.AutonShoot;

public class UI {
    public SendableChooser<delay> delayChooser = new SendableChooser<>();
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
//        SmartDashboard.putNumber("Shooter/P", 0.3);
//        SmartDashboard.putNumber("Shooter/I", 0);
//        SmartDashboard.putNumber("Shooter/D", 0);
//        SmartDashboard.putNumber("Shooter/Reference", 3000);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());


        delayChooser.setDefaultOption("0", delay.DELAY_0);
        delayChooser.addOption("1", delay.DELAY_1);
        delayChooser.addOption("2", delay.DELAY_2);
        delayChooser.addOption("3", delay.DELAY_3);
        delayChooser.addOption("4", delay.DELAY_4);
        delayChooser.addOption("5", delay.DELAY_5);
        delayChooser.addOption("6", delay.DELAY_6);
        delayChooser.addOption("7", delay.DELAY_7);
        delayChooser.addOption("8", delay.DELAY_8);
        delayChooser.addOption("9", delay.DELAY_9);
        delayChooser.addOption("10", delay.DELAY_10);
        delayChooser.addOption("11", delay.DELAY_11);
        delayChooser.addOption("12", delay.DELAY_12);
        delayChooser.addOption("13", delay.DELAY_13);
        delayChooser.addOption("14", delay.DELAY_14);
        delayChooser.addOption("15", delay.DELAY_15);
        delayChooser.addOption("16", delay.DELAY_16);
        delayChooser.addOption("17", delay.DELAY_17);
        delayChooser.addOption("18", delay.DELAY_18);
        delayChooser.addOption("19", delay.DELAY_19);
        delayChooser.addOption("20", delay.DELAY_20);
        SmartDashboard.putData("Delay Chooser", delayChooser);
    }

    /**
     * Widgets which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     SmartDashboard.putString("Elevator", String.format("%11.2f", Elevator.getPosition());
     */
    public void update() {
        SmartDashboard.putNumber("Shooter/Shooter 1 Speed", Robot.shooter.getLeftMotorSpeed());
        SmartDashboard.putNumber("Shooter/Shooter 2 Speed", Robot.shooter.getMiddleMotorSpeed());
        SmartDashboard.putNumber("Shooter/Shooter 3 Speed", Robot.shooter.getRightMotorSpeed());
        SmartDashboard.putNumber("Shooter/Shooter 1 Current", Robot.shooter.getLeftMotorCurrent());
        SmartDashboard.putNumber("Shooter/Shooter 1 Output", Robot.shooter.getLeftMotorOutput());
        SmartDashboard.putNumber("Shooter/Shooter 2 Output", Robot.shooter.getMiddleMotorOutput());
        SmartDashboard.putNumber("Shooter/Shooter 3 Output", Robot.shooter.getRightMotorOutput());
        SmartDashboard.putNumber("Indexer/Bottom Speed", Robot.indexer.getBottomIndexerSpeed());
        SmartDashboard.putNumber("Indexer/Top Speed", Robot.indexer.getTopIndexerSpeed());
        SmartDashboard.putNumber("Collector/Collector Output", Robot.collectMotor.getCollectOutput());
        SmartDashboard.putNumber("Collector/Collector Speed", Robot.collectMotor.getCollectSpeed());
        SmartDashboard.putNumber("Collector/Pivot Output", Robot.pivotMotor.getPivotOutputPercent());
        SmartDashboard.putNumber("Collector/Pivot Speed", Robot.pivotMotor.getPivotSpeed());
        SmartDashboard.putNumber("Collector/Pivot Position Numeric", Robot.pivotMotor.getPosition());
        SmartDashboard.putString("Collector/Pivot Position", String.format("%11.4f", Robot.pivotMotor.getPosition()));
        SmartDashboard.putBoolean("Diagnostics/Motor Temp", Robot.diagnostics.anyMotorTempHot());
        SmartDashboard.putBoolean("Diagnostics/Swerve Motor Hot", Robot.diagnostics.swerveMotorTempHot());
        SmartDashboard.putBoolean("Diagnostics/Mechanism Motor Hot", Robot.diagnostics.mechanismMotorTempHot());
        SmartDashboard.putBoolean("Competition", Robot.isCompetition);
        SmartDashboard.putString("ShiftTime", String.format("%11.1f", Robot.shiftTime));
        SmartDashboard.putString("Shift",Robot.matchShift);
        SmartDashboard.putString("Rot Offset", String.format("%11.0f", AutonShoot.hubCommand.getRotationOffset() * -1));
        SmartDashboard.putString("RPM Offset", String.format("%11.0f", AutonShoot.shootCommand.getRPMOffset()));
        SmartDashboard.putBoolean("Manual", AutonShoot.shootCommand.getManualMode());
        updateTemps();
    }

    public void updateTemps(){
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
