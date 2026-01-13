package team.gif.robot.subsystems.drivers.swerve;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConstants;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotor;

public class SparkMAXTurnMotor implements TurnMotor {
    private final SparkMax motor;
    private final double positionConversion;
    private final double velocityConversion;

    /**
     * Creates a sparkmax motor (NEO) to be used
     * as a turn motor on a swerve drivetrain.
     * This should never be used except for that purpose.
     * @param id the CAN ID of the motor controller
     */
    public SparkMAXTurnMotor(int id, SwerveConstants constants) {
        motor = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
        positionConversion = constants.TURNING_ENCODER_ROT_TO_RAD;
        velocityConversion = constants.TURNING_ENCODER_RPM_2_RAD_PER_SECOND;
    }

    public void configure(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        config.inverted(inverted);
        config.voltageCompensation(12);
        config.encoder.positionConversionFactor(positionConversion);
        config.encoder.velocityConversionFactor(velocityConversion);
        config.smartCurrentLimit(70, 50);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public double getOutput() {
        return motor.getAppliedOutput();

    }

    public double getVoltage() {
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    public void set(double percentOutput) {
        motor.set(percentOutput);

    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
