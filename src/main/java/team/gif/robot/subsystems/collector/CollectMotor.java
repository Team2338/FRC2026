// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems.collector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class CollectMotor extends SubsystemBase {

    private final TalonFX collectorMotor;
    private TalonFXConfiguration collectorMotorConfig;
    public VelocityVoltage velocityVoltage;

    public CollectMotor() {
        collectorMotor = new TalonFX(RobotMap.Collector.COLLECT_MOTOR_ID);

        setConfig();

        velocityVoltage = new VelocityVoltage(0).withSlot(0);
    }


    @Override
    public void periodic() {
        double netP = SmartDashboard.getNumber("Collector/PID/Collect P", 0);
        double netI = SmartDashboard.getNumber("Collector/PID/Collect I", 0);
        double netD = SmartDashboard.getNumber("Collector/PID/Collect D", 0);

        double currP = collectorMotorConfig.Slot0.kP;
        double currI = collectorMotorConfig.Slot0.kI;
        double currD = collectorMotorConfig.Slot0.kD;

        if(netP != currP || netI != currI || netD != currD) {
            collectorMotorConfig.Slot0.kP = netP;
            collectorMotorConfig.Slot0.kI = netI;
            collectorMotorConfig.Slot0.kD = netD;
            setCollectorMotorConfig(collectorMotorConfig);
        }
    }

    public void runCollectorPercent(double percent){
        collectorMotor.set(percent);
    }

    public void runCollector(double rpm) {
        collectorMotor.setControl(velocityVoltage.withVelocity(-rpm/60));
    }

    public double getCollectOutput() {
        return collectorMotor.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getCollectSpeed() {
        return Math.abs(collectorMotor.getVelocity().getValueAsDouble() * 60);
    }

    public void stopMotor() {collectorMotor.stopMotor();}

    public boolean isOverTemp() {
        return collectorMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.COLLECTOR_MOTOR_TEMP_WARNING_CELSIUS;
    }

    public void setCollectorMotorConfig(TalonFXConfiguration collectorMotorConfig) {
        collectorMotor.getConfigurator().apply(collectorMotorConfig);
    }

    public void setConfig() {
        collectorMotorConfig = new TalonFXConfiguration(); //Factory defaults are applied to new config object
        collectorMotorConfig.Slot0.kP = 0.35;
        collectorMotorConfig.Slot0.kI = 0;
        collectorMotorConfig.Slot0.kD = 0;

        collectorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        collectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        collectorMotor.getConfigurator().apply(collectorMotorConfig);
    }

}
