// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.RobotMap;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {

    public TalonFX shooter1;
    public TalonFX shooter2;
    public TalonFX shooter3;
    public TalonFXConfiguration config = new TalonFXConfiguration();
    public VelocityVoltage velocityVoltage;

    /** Creates a new ExampleSubsystem. */
    public Shooter() {
       shooter1 = new TalonFX(RobotMap.Shooter.SHOOTER_1);
       shooter2 = new TalonFX(RobotMap.Shooter.SHOOTER_2);
       shooter3 = new TalonFX(RobotMap.Shooter.SHOOTER_3);

       config.Slot0.kP = 0;
       config.Slot0.kI = 0;
       config.Slot0.kD = 0;
       config.Slot0.kS = 0.12278;
       config.Slot0.kV = 0.11522;
       config.Slot0.kA = 0.0078728;

       shooter1.getConfigurator().apply(config);
       shooter2.getConfigurator().apply(config);
       shooter3.getConfigurator().apply(config);

       Follower follower = new Follower(shooter1.getDeviceID(), MotorAlignmentValue.Opposed);

//       shooter2.setControl(follower);
//       shooter3.setControl(follower);

       velocityVoltage = new VelocityVoltage(0).withSlot(0);
    }

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
            config.Slot0.kS = 0.12278;
            config.Slot0.kV = 0.11522;
            config.Slot0.kA = 0.0078728;
            setConfig(config);
            velocityVoltage = new VelocityVoltage(0).withSlot(0);
        }

    }

    public void runShooterPercent(double percent) {
        shooter1.set(percent);
        System.out.println(percent);
    }

    public void runShooterVoltage(double voltage) {
        shooter1.setVoltage(voltage);
    }

    public void runShooter(double rpm) {

        shooter1.setControl(velocityVoltage.withVelocity(rpm/60));
        shooter2.setControl(velocityVoltage.withVelocity(rpm/60));
        shooter3.setControl(velocityVoltage.withVelocity(rpm/60));

    }

    public double getSpeed() {
        return shooter1.getVelocity().getValueAsDouble() * 60;
    }

    public double getSpeed3() {
        return shooter3.getVelocity().getValueAsDouble() * 60;
    }

    public double getSpeed2() {
        return shooter2.getVelocity().getValueAsDouble() * 60;
    }

    public double getCurrent() {
        return shooter1.getSupplyCurrent().getValueAsDouble();
    }

    public double getOutput() {
        return shooter1.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getOutput2() {
        return shooter2.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getOutput3() {
        return shooter3.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getVoltage() {
        return shooter1.getMotorVoltage().getValueAsDouble();
    }

    public void stopMotor() {
        shooter1.stopMotor();
        shooter2.stopMotor();
        shooter3.stopMotor();
    }

    public void setConfig(TalonFXConfiguration config) {
        shooter1.getConfigurator().apply(config);
        shooter3.getConfigurator().apply(config);
    }

    private void sysIDVoltage(Voltage volt) {
        runShooterVoltage(volt.baseUnitMagnitude());
    }

    private void sysIDLog(SysIdRoutineLog log) {
        MutVoltage voltMut = Volts.mutable(0);
        MutAngle posMut = Rotations.mutable(0);
        MutAngularVelocity vMut= RotationsPerSecond.mutable(0);

        log.motor("Shooter")
                .voltage(voltMut.mut_replace(getVoltage(), Volts))
                .angularVelocity(vMut.mut_replace(shooter1.getVelocity().getValueAsDouble(), RotationsPerSecond))
                .angularPosition(posMut.mut_replace(shooter1.getPosition().getValueAsDouble(), Rotations));
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
