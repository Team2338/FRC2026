// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems.collector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class CollectMotor extends SubsystemBase {

    private final TalonFX collectorMotor;
    private final TalonFX collectorMotor2;
    private TalonFXConfiguration config;
    private DutyCycleOut dutyCycleOut;

    public CollectMotor() {
        collectorMotor = new TalonFX(RobotMap.Collector.COLLECT_MOTOR_ID);
        collectorMotor2 = new TalonFX(RobotMap.Collector.COLLECTOR_MOTOR_2_ID);

        Follower follower = new Follower(collectorMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        collectorMotor2.setControl(follower);

        setConfig();

        dutyCycleOut = new DutyCycleOut(0);
    }

    public void runCollectorPercent(double percent){
        dutyCycleOut.withOutput(percent).withEnableFOC(false);
        collectorMotor.setControl(dutyCycleOut);
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

    public void setConfig(TalonFXConfiguration config) {
        collectorMotor.getConfigurator().apply(config);
    }

    public void setConfig() {
        config = new TalonFXConfiguration(); //Factory defaults are applied to new config object
        config.Slot0.kS = 0.12695;
        config.Slot0.kV = 0.11474;
        config.Slot0.kA = 0.0058193;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // We want brake, but CTRE suggested taking out of brake mode // TODO

        collectorMotor.getConfigurator().apply(config);
    }

    private void runVoltage(double voltage) {
        collectorMotor.setVoltage(voltage);
    }

    private double getVoltage() {
        return collectorMotor.getMotorVoltage().getValueAsDouble();
    }

    private void sysIDVoltage(Voltage volt) {
        runVoltage(volt.baseUnitMagnitude());
    }

    private void sysIDLog(SysIdRoutineLog log) {
        MutVoltage voltMut = Volts.mutable(0);
        MutAngle posMut = Rotations.mutable(0);
        MutAngularVelocity vMut= RotationsPerSecond.mutable(0);

        log.motor("Shooter")
                .voltage(voltMut.mut_replace(getVoltage(), Volts))
                .angularVelocity(vMut.mut_replace(collectorMotor.getVelocity().getValueAsDouble(), RotationsPerSecond))
                .angularPosition(posMut.mut_replace(collectorMotor.getPosition().getValueAsDouble(), Rotations));
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
