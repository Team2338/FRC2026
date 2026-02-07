// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Collector extends SubsystemBase {

    private final TalonFX collectorMotor;
    private final TalonFX pivotMotor;
    public TalonFXConfiguration config = new TalonFXConfiguration();
    public VelocityVoltage velocityVoltage;

    public Collector() {
        collectorMotor = new TalonFX(RobotMap.Collector.COLLECT_MOTOR);
        pivotMotor = new TalonFX(RobotMap.Collector.PIVOT_MOTOR);

        config.Slot0.kP = 0.35;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        setConfig(config);

        velocityVoltage = new VelocityVoltage(0).withSlot(0);

    }

    @Override
    public void periodic() {

        double netP = SmartDashboard.getNumber("Collector/PID/Collect P", 0);
        double netI = SmartDashboard.getNumber("Collector/PID/Collect I", 0);
        double netD = SmartDashboard.getNumber("Collector/PID/Collect D", 0);

        double currP = config.Slot0.kP;
        double currI = config.Slot0.kI;
        double currD = config.Slot0.kD;


        if(netP != currP || netI != currI || netD != currD) {
            config.Slot0.kP = netP;
            config.Slot0.kI = netI;
            config.Slot0.kD = netD;
            setConfig(config);
        }

    }

    public void runCollectorPercent(double percent){
        collectorMotor.set(percent);
    }

    public void runCollectorVoltage(double volts){
        collectorMotor.setVoltage(volts);
    }

    public void runCollector(double rpm) {
        collectorMotor.setControl(velocityVoltage.withVelocity(-rpm/60));
    }

    public void runPivotPercent(double percent) {
        pivotMotor.set(percent);
    }

    public void runPivotVoltage(double volts){
        pivotMotor.setVoltage(volts);
    }

    public double getCollectOutput() {
        return collectorMotor.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getCollectSpeed() {
        return Math.abs(collectorMotor.getVelocity().getValueAsDouble() * 60);
    }

    public double getPivotOutput() {
        return pivotMotor.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getPivotSpeed() {
        return pivotMotor.getVelocity().getValueAsDouble() * 60;
    }

    public void stopMotor() {
        collectorMotor.stopMotor();
    }

    public void setConfig(TalonFXConfiguration config) {
        collectorMotor.getConfigurator().apply(config);
    }

}
