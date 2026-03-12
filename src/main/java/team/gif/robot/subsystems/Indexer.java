// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {

    public TalonFX bottomIndexerMotor;
    public TalonFX topIndexerMotor;
    public TalonFXConfiguration bottomIndexerConfig = new TalonFXConfiguration();
    public TalonFXConfiguration topIndexerConfig = new TalonFXConfiguration();

    public Indexer() {
       bottomIndexerMotor = new TalonFX(RobotMap.Shooter.BOTTOM_INDEXER_MOTOR_ID);
       topIndexerMotor = new TalonFX(RobotMap.Shooter.TOP_INDEXER_MOTOR_ID);

       bottomIndexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       bottomIndexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
       topIndexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
       topIndexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

       bottomIndexerMotor.getConfigurator().apply(bottomIndexerConfig);
       topIndexerMotor.getConfigurator().apply(topIndexerConfig);
    }

    public void run(double bottomIndexerMotorPercent, double topIndexerMotorPercent) {
        bottomIndexerMotor.set(bottomIndexerMotorPercent);
        topIndexerMotor.set(topIndexerMotorPercent);
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
}
