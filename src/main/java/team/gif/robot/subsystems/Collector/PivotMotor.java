package team.gif.robot.subsystems.Collector;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;

public class PivotMotor extends SubsystemBase {

    private final TalonFX pivotMotor;
    private TalonFXSConfiguration config;

    public PivotMotor(){
        pivotMotor = new TalonFX(RobotMap.Collector.PIVOT_MOTOR);
    }

    public void runPivotPercent(double percent) {
        pivotMotor.set(percent);
    }

    public void runPivotVoltage(double volts){
        pivotMotor.setVoltage(volts);
    }

    public double getPivotOutput() {
        return pivotMotor.getMotorVoltage().getValueAsDouble() / 12;
    }

    public double getPivotSpeed() {
        return pivotMotor.getVelocity().getValueAsDouble() * 60;
    }

    public double getPosition(){return pivotMotor.getPosition().getValueAsDouble();}

    public void zeroEncoder(){pivotMotor.setPosition(0);}

    private void setConfig(){
        config = new TalonFXSConfiguration();

        config.MotorOutput.Inverted = Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    }

    //Change position value later after testing
    //public void deployedEncoder(){pivotMotor.setPosition(0);}
}
