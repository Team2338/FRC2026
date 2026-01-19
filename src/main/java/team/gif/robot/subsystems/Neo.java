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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.RobotMap;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Neo extends SubsystemBase {
    public SparkFlex spark;
    public SparkClosedLoopController pid;
    public SparkFlexConfig config = new SparkFlexConfig();
    public double p = 0;
    public double i = 0;
    public double d = 0;

    public Neo() {
        spark = new SparkFlex(RobotMap.SPARK_ID, SparkLowLevel.MotorType.kBrushless);
        pid = spark.getClosedLoopController();
        config.closedLoop.pid(p, d, d);
        config.inverted(true);



        setConfig(config);
    }

    public void run(double percent) {
        spark.set(percent);
    }

    public void setVoltage(double voltage) {
        spark.setVoltage(voltage);
    }

    public void setReference(double reference) {
        System.out.println(reference);
        pid.setSetpoint(reference, SparkBase.ControlType.kVelocity);
        System.out.println("set ref");
    }

    private void setConfig(SparkFlexConfig newConfig){
        spark.configureAsync(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        System.out.println("update2");
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
            config.closedLoop.iMaxAccum(10000000);
            setConfig(config);
            System.out.println("update");
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

    public double getVoltage() {
        return spark.getAppliedOutput() * spark.getBusVoltage();
    }

    private void sysIDVoltage(Voltage volt) {
        spark.setVoltage(volt.baseUnitMagnitude());
    }

    private void sysIDLog(SysIdRoutineLog log) {
        MutVoltage voltMut = Volts.mutable(0);
        MutAngle posMut = Rotations.mutable(0);
        MutAngularVelocity vMut= RotationsPerSecond.mutable(0);

        log.motor("Shooter")
                .voltage(voltMut.mut_replace(getVoltage(), Volts))
                .angularVelocity(vMut.mut_replace(getSpeed(), RotationsPerSecond))
                .angularPosition(posMut.mut_replace(spark.getEncoder().getPosition(), Rotations));
    }


    public SysIdRoutine getSysID() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(this::sysIDVoltage, this::sysIDLog, this)
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return getSysID().quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return getSysID().dynamic(direction);
    }


}
