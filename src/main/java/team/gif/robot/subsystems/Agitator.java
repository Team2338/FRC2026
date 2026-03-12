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

public class Agitator extends SubsystemBase {

    private final TalonFX agitatorMotor;
    private TalonFXConfiguration agitatorMotorConfig;

    public Agitator() {
        agitatorMotor = new TalonFX(RobotMap.Agitator.AGITATOR_MOTOR_ID);
        setConfig();
    }

    public void setPercent(double percent) {
        agitatorMotor.set(percent);
    }

    public void stopMotor() {
        agitatorMotor.stopMotor();
    }

    public boolean isOverTemp() {
        return agitatorMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.AGITATOR_MOTOR_TEMP_WARNING_CELSIUS;
    }

    public void setConfig(){
        agitatorMotorConfig = new TalonFXConfiguration(); //Factory defaults are applied to new config object
        agitatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        agitatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        agitatorMotor.getConfigurator().apply(agitatorMotorConfig);
    }
}
