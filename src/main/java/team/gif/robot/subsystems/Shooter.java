// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {

    public TalonFX shooterLeftMotor;
    public TalonFX shooterMiddleMotor;
    public TalonFX shooterRightMotor;
    public VelocityVoltage velocityVoltage;
    public double RPMSetpointLower;
    public double RPMSetpointHigher;

    /** Creates a new ExampleSubsystem. */
    public Shooter() {
       shooterLeftMotor = new TalonFX(RobotMap.Shooter.LEFT_SHOOTER_MOTOR_ID);
       shooterMiddleMotor = new TalonFX(RobotMap.Shooter.MIDDLE_SHOOTER_MOTOR_ID);
       shooterRightMotor = new TalonFX(RobotMap.Shooter.RIGHT_SHOOTER_MOTOR_ID);

       setConfig();

       velocityVoltage = new VelocityVoltage(0).withSlot(0);
    }

    /*
    @Override
    public void periodic() {

        double netP = SmartDashboard.getNumber("COLLECTOR/PID/P", 0);
        double netI = SmartDashboard.getNumber("COLLECTOR/PID/I", 0);
        double netD = SmartDashboard.getNumber("COLLECTOR/PID/D", 0);

        double currP = config.Slot0.kP;
        double currI = config.Slot0.kI;
        double currD = config.Slot0.kD;


        if(netP != currP || netI != currI || netD != currD) {
            config.Slot0.kP = netP;
            config.Slot0.kI = netI;
            config.Slot0.kD = netD;
            config.Slot0.kS = 0.12278;
            config.Slot0.kV = 0.11522;
            config.Slot0.kA = 0.0078728;
            setConfig(config);
            velocityVoltage = new VelocityVoltage(0).withSlot(0);
        }

    }
    */

    public void runLeftShooterPercent(double percent) {
        shooterLeftMotor.set(percent);
    }

    public void runLeftShooterVoltage(double voltage) {
        shooterLeftMotor.setVoltage(voltage);
    }

    public void runShooter(double rpm) {
        shooterLeftMotor.setControl(velocityVoltage.withVelocity(rpm/60));
        shooterMiddleMotor.setControl(velocityVoltage.withVelocity(rpm/60));
        shooterRightMotor.setControl(velocityVoltage.withVelocity(rpm/60));
    }

    public boolean isShooterReady() {
        RPMSetpointLower = ShotCalculator.getShotRPM() - 100;
        RPMSetpointHigher = ShotCalculator.getShotRPM() + 100;

        return (RPMSetpointLower <= getLeftMotorSpeed() && getLeftMotorSpeed() <= RPMSetpointHigher)
                && (RPMSetpointLower <= getMiddleMotorSpeed() && getMiddleMotorSpeed() <= RPMSetpointHigher)
                && (RPMSetpointLower <= getRightMotorSpeed() && getRightMotorSpeed() <= RPMSetpointHigher);
    }

    public double getLeftMotorSpeed() {
        return shooterLeftMotor.getVelocity().getValueAsDouble() * 60;
    }

    public double getRightMotorSpeed() {
        return shooterRightMotor.getVelocity().getValueAsDouble() * 60;
    }

    public double getMiddleMotorSpeed() {
        return shooterMiddleMotor.getVelocity().getValueAsDouble() * 60;
    }

    public double getLeftMotorCurrent() {
        return shooterLeftMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getLeftMotorOutput() {
        return shooterLeftMotor.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getMiddleMotorOutput() {
        return shooterMiddleMotor.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getRightMotorOutput() {
        return shooterRightMotor.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getLeftMotorVoltage() {
        return shooterLeftMotor.getMotorVoltage().getValueAsDouble();
    }

    public void stopMotors() {
        shooterLeftMotor.stopMotor();
        shooterMiddleMotor.stopMotor();
        shooterRightMotor.stopMotor();
    }


    public boolean isLeftMotorOverTemp() {
        return shooterLeftMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.SHOOTER_MOTOR_TEMP_WARNING_CELSIUS;
    }

    public boolean isMiddleMotorOverTemp() {
        return shooterMiddleMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.SHOOTER_MOTOR_TEMP_WARNING_CELSIUS;
    }

    public boolean isRightMotorOverTemp() {
        return shooterRightMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.SHOOTER_MOTOR_TEMP_WARNING_CELSIUS;
    }

    public boolean isRPMSufficient(double targetRPM) {
        return getLeftMotorSpeed() > targetRPM && getMiddleMotorSpeed() > targetRPM && getRightMotorSpeed() > targetRPM;
    }

    public void setConfig() {
        TalonFXConfiguration shooterLeftConfig = new TalonFXConfiguration(); //Factory defaults are applied to new config object;
        TalonFXConfiguration shooterMiddleConfig; //will be cloned
        TalonFXConfiguration shooterRightConfig; //will be cloned

        shooterLeftConfig.Slot0.kP = 0;
        shooterLeftConfig.Slot0.kI = 0;
        shooterLeftConfig.Slot0.kD = 0;
        shooterLeftConfig.Slot0.kS = 0.12278;
        shooterLeftConfig.Slot0.kV = 0.11522;
        shooterLeftConfig.Slot0.kA = 0.0078728;

        shooterMiddleConfig = shooterLeftConfig.clone();
        shooterRightConfig = shooterLeftConfig.clone();

        shooterLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterMiddleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMiddleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterLeftMotor.getConfigurator().apply(shooterLeftConfig);
        shooterMiddleMotor.getConfigurator().apply(shooterMiddleConfig);
        shooterRightMotor.getConfigurator().apply(shooterRightConfig);
    }

    private void sysIDVoltage(Voltage volt) {
        runLeftShooterVoltage(volt.baseUnitMagnitude());
    }

    private void sysIDLog(SysIdRoutineLog log) {
        MutVoltage voltMut = Volts.mutable(0);
        MutAngle posMut = Rotations.mutable(0);
        MutAngularVelocity vMut= RotationsPerSecond.mutable(0);

        log.motor("Shooter")
                .voltage(voltMut.mut_replace(getLeftMotorVoltage(), Volts))
                .angularVelocity(vMut.mut_replace(shooterLeftMotor.getVelocity().getValueAsDouble(), RotationsPerSecond))
                .angularPosition(posMut.mut_replace(shooterLeftMotor.getPosition().getValueAsDouble(), Rotations));
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
