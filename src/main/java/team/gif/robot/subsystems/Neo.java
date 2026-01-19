// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Neo extends SubsystemBase {
    public SparkFlex spark;
    public SparkClosedLoopController pid;
    public SparkFlexConfig config;
    public double p = 0;
    public double i = 0;
    public double d = 0;

    public Neo() {
        spark = new SparkFlex(RobotMap.SPARK_ID, SparkLowLevel.MotorType.kBrushless);
        pid = spark.getClosedLoopController();
        config.closedLoop.pid(p, d, d);

        setConfig(config);
    }

    public void run(double percent) {
        spark.set(percent);
    }

    public void setVoltage(double voltage) {
        spark.setVoltage(voltage);
    }

    public void setReference(double reference) {
        pid.setSetpoint(reference, SparkBase.ControlType.kVelocity);
    }

    private void setConfig(SparkFlexConfig newConfig){
        spark.configureAsync(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setP(double newp) {
       config.closedLoop.p(newp);
       setConfig(config);
    }

    public void setI(double newi) {
        config.closedLoop.i(newi);
        setConfig(config);
    }

    public void setD(double newd) {
        config.closedLoop.d(newd);
        setConfig(config);
    }

    @Override
    public void periodic() {
        double netP = SmartDashboard.getNumber("PID/P", 0);
        double netI = SmartDashboard.getNumber("PID/I", 0);
        double netD = SmartDashboard.getNumber("PID/D", 0);

        if(netP != p || netI != i || netD != d) {
            p = netP;
            i = netI;
            d = netD;
            config.closedLoop.pid(netP, netI, netD);
            setConfig(config);
        }
    }

    public void stop() {
        spark.set(0);
    }

    public double getSpeed() {
        return spark.getEncoder().getVelocity();
    }

    public double getCurrent() {
        return spark.getOutputCurrent();
    }
}
