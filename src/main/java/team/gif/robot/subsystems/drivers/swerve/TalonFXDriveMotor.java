package team.gif.robot.subsystems.drivers.swerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import team.gif.robot.subsystems.drivers.swerve.utilities.DriveMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConstants;

public class TalonFXDriveMotor implements DriveMotor {
    private final TalonFX motor;
    private final double positionConversion;
    private final double velocityConversion;

    public TalonFXDriveMotor(int motorID, SwerveConstants constants) {
        motor = new TalonFX(motorID);
        positionConversion = constants.DRIVE_ENCODER_ROT_2_METER;
        velocityConversion = constants.DRIVE_ENCODER_RPM_2_METER_PER_SEC;
    }

    /**
     * Configures a TalonFX to be a drive motor on a swerve drivetrain.
     * Sets empty TalonFX config, and sets it to brake mode.
     */
    public void configure(boolean inverted) {
        TalonFXConfigurator talonFXConfig = motor.getConfigurator();

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        talonFXConfig.apply(motorConfigs);
    }

    public double getTemp() {
        return motor.getDeviceTemp().getValueAsDouble();
    }


    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() * positionConversion;
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble() * velocityConversion;
    }


    public double getOutput() {
        return motor.get();
    }

    public void set(double percentOutput) {
        motor.set(percentOutput);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public void resetEncoder() {
        motor.setPosition(0);
    }

}
