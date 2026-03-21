// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {

    public TalonFX bottomIndexerMotor;
    public TalonFX topIndexerMotor;
    public TalonFXConfiguration topIndexerConfig;
    public VelocityVoltage velocityVoltage;

    public Indexer() {
       bottomIndexerMotor = new TalonFX(RobotMap.Shooter.BOTTOM_INDEXER_MOTOR_ID);
       topIndexerMotor = new TalonFX(RobotMap.Shooter.TOP_INDEXER_MOTOR_ID);

       setConfig();

       velocityVoltage = new VelocityVoltage(0).withSlot(0);
    }

    @Override
    public void periodic() {
        double netP = SmartDashboard.getNumber("Indexer/PID/T_Index P", Constants.Indexer.TOP_INDEXER_P);
        double netI = SmartDashboard.getNumber("Indexer/PID/T_Index I", Constants.Indexer.TOP_INDEXER_I);
        double netD = SmartDashboard.getNumber("Indexer/PID/T_Index D", Constants.Indexer.TOP_INDEXER_D);

        double currP = topIndexerConfig.Slot0.kP;
        double currI = topIndexerConfig.Slot0.kI;
        double currD = topIndexerConfig.Slot0.kD;

        if(netP != currP || netI != currI || netD != currD) {
            topIndexerConfig.Slot0.kP = netP;
            topIndexerConfig.Slot0.kI = netI;
            topIndexerConfig.Slot0.kD = netD;
            setConfig();
        }
    }

    public void runPercent(double bottomIndexerMotorPercent, double topIndexerMotorPercent) {
        bottomIndexerMotor.set(bottomIndexerMotorPercent);
        topIndexerMotor.set(topIndexerMotorPercent);
    }

    public void runTopIndexer(double topRPM){
        topIndexerMotor.setControl(velocityVoltage.withVelocity(topRPM/60));
    }

    public double getBottomIndexerSpeed() {
        return bottomIndexerMotor.getVelocity().getValueAsDouble();
    }

    public double getBottomIndexerCurrent() {
        return bottomIndexerMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getBottomIndexerOutput() {
        return bottomIndexerMotor.getBridgeOutput().getValueAsDouble();
    }

    public double getTopIndexerSpeed() {
        return topIndexerMotor.getVelocity().getValueAsDouble();
    }

    public double getTopIndexerCurrent() {
        return topIndexerMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getTopIndexerOutput() {
        return topIndexerMotor.getBridgeOutput().getValueAsDouble();
    }

    public void stopMotors() {
        bottomIndexerMotor.stopMotor();
        topIndexerMotor.stopMotor();
    }

    public boolean isBottomIndexerMotorOverTemp() {
        return bottomIndexerMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.BOTTOM_INDEXER_MOTOR_WARNING_CELSIUS;
    }

    public boolean isTopIndexerMotorOverTemp() {
        return topIndexerMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.TOP_INDEXER_MOTOR_TEMP_WARNING_CELSIUS;
    }

    public void setBottomIndexerConfig(TalonFXConfiguration bottomIndexerConfig) {
        bottomIndexerMotor.getConfigurator().apply(bottomIndexerConfig);
    }

    public void setConfig() {
        TalonFXConfiguration bottomIndexerConfig = new TalonFXConfiguration(); //Factory defaults are applied to new config object
        topIndexerConfig = new TalonFXConfiguration();

        bottomIndexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        bottomIndexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        topIndexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topIndexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        topIndexerConfig.Slot0.kP = 0.0;
        topIndexerConfig.Slot0.kI = 0.0;
        topIndexerConfig.Slot0.kD = 0.0;

        bottomIndexerMotor.getConfigurator().apply(bottomIndexerConfig);
        topIndexerMotor.getConfigurator().apply(topIndexerConfig);
    }
}
