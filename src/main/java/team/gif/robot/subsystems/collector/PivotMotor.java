package team.gif.robot.subsystems.collector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

public class PivotMotor extends SubsystemBase {

    private final TalonFX pivotMotor;
    private final CANcoder pivotEncoder;

    public PivotMotor(){
        pivotMotor = new TalonFX(RobotMap.Collector.PIVOT_MOTOR_ID);
        pivotEncoder = new CANcoder(RobotMap.Collector.PIVOT_ENCODER_ID);

        setConfig();
    }

    /**
     * Positive values deploy the collector
     * @param percent percent of power to run pivot motor
     */
    public void runPivotPercent(double percent) {
        pivotMotor.set(percent);
    }

    public double getPivotOutput() {
        return pivotMotor.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getPivotSpeed() {
        return pivotMotor.getVelocity().getValueAsDouble() * 60;
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public void zeroEncoder() {
        pivotMotor.setPosition(0);
    }

    public void deployedEncoder() {
        pivotMotor.setPosition(Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS);
    }

    public double getAbsEncoderPos(){
       return pivotEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public boolean atSetPoint(double desiredSetpoint) {
        return Math.abs(getPosition() - desiredSetpoint) <= Constants.Collector.PIVOT_POSITION_TOLERANCE;
    }

    private void setConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration(); //Factory defaults are applied to new config object

        config.MotorOutput.Inverted = CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.FeedbackRemoteSensorID = RobotMap.Collector.PIVOT_ENCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Collector.PIVOT_SOFT_LIMIT_UP_ENCODER_POS;

        pivotMotor.getConfigurator().apply(config);
    }

    public boolean isOverTemp() {
        return pivotMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.PIVOT_MOTOR_TEMP_WARNING_CELSIUS;
    }

    public void stopMotor() {
        pivotMotor.stopMotor();
    }
}
