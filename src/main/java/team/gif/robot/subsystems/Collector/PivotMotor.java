package team.gif.robot.subsystems.Collector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

public class PivotMotor extends SubsystemBase {

    private final TalonFX pivotMotor;
    private TalonFXConfiguration config;

    public PivotMotor() {
        pivotMotor = new TalonFX(RobotMap.Collector.PIVOT_MOTOR);

        setConfig();
    }

    public void runPivotPercent(double percent) {
        pivotMotor.set(percent);
    }

    public void runPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
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

    public void deployedEncoder(double setpoint){
        pivotMotor.setPosition(setpoint);
    }

    public boolean atSetPoint(double desiredSetpoint) {
        return Math.abs(getPosition() - desiredSetpoint) <= Constants.Collector.AUTO_COLLECTOR_PIVOT_TOLERANCE;
    }

    private void setConfig() {
        config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Collector.PIVOT_SOFT_LIMIT_UP_ENCODER_POS;
        pivotMotor.getConfigurator().apply(config);
    }

    public boolean isOverTemp() {
        return pivotMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.PIVOT_SAFE_MOTOR_TEMP;
    }

    public boolean isStalling(){
        return pivotMotor.getTorqueCurrent().getValueAsDouble() >= Constants.Collector.PIVOT_STALL_CURRENT_THRESHOLD;
    }

    //Change position value later after testing
    //public void deployedEncoder(){pivotMotor.setPosition(0);}
}
