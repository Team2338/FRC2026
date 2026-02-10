package team.gif.robot.subsystems.Collector;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class PivotMotor extends SubsystemBase {

    private final TalonFX pivotMotor;
    //public TalonFXConfiguration config = new TalonFXConfiguration(); - shouldn't need this because no PID atm

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
    
}
