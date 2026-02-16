package team.gif.robot.subsystems.Collector;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class PivotMotor extends SubsystemBase {

    private final TalonFX pivotMotor;

    public PivotMotor(){
        pivotMotor = new TalonFX(RobotMap.Collector.PIVOT_MOTOR);
    }

    public void runPivotPercent(double percent) {
        pivotMotor.set(percent);
        System.out.println(percent);
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

    public boolean isOverTemp() {
        return pivotMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.PIVOT_SAFE_MOTOR_TEMP;
    }

    //Change position value later after testing
    //public void deployedEncoder(){pivotMotor.setPosition(0);}
}
