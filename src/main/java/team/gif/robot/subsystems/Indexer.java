// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {

    public TalonFX indexer;
    public TalonFX indexer2;
    public TalonFXConfiguration config = new TalonFXConfiguration();
    public TalonFXConfiguration config2 = new TalonFXConfiguration();
//    public SparkFlexConfig sparkConfig = new SparkFlexConfig();

    /** Creates a new ExampleSubsystem. */
    public Indexer() {
       indexer = new TalonFX(RobotMap.Shooter.INDEXER);
       indexer2 = new TalonFX(RobotMap.Shooter.INDEXER_2);

       config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
       config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

       indexer.getConfigurator().apply(config);
       indexer2.getConfigurator().apply(config2);
    }
/*
    @Override
    public void periodic() {

        double netP = SmartDashboard.getNumber("PID/P", 0);
        double netI = SmartDashboard.getNumber("PID/I", 0);
        double netD = SmartDashboard.getNumber("PID/D", 0);

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

 */

    public void runPercent(double stage1, double stage2) {
        indexer.set(stage2);
        indexer2.set(stage1);
    }

    public void runVoltage(double voltage) {
        indexer.setVoltage(voltage);
        indexer2.setVoltage(voltage);
    }

//    public void run(double rpm) {}

    public double getSpeed() {
        return indexer.getVelocity().getValueAsDouble();
    }

    public double getCurrent() {
        return indexer.getSupplyCurrent().getValueAsDouble();
    }

    public double getOutput() {
        return indexer.getBridgeOutput().getValueAsDouble();
    }

    public void stopMotor() {
        indexer.stopMotor();
        indexer2.stopMotor();
    }

    public boolean isIndexOneOverTemp() {
        return indexer.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.INDEX_SAFE_MOTOR_TEMP;
    }

    public boolean isIndexTwoOverTemp() {
        return indexer2.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.INDEX_SAFE_MOTOR_TEMP;
    }

    public void setConfig(TalonFXConfiguration config) {
        indexer.getConfigurator().apply(config);
    }

}
