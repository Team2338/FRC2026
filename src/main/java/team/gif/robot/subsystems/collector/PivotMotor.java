package team.gif.robot.subsystems.collector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class PivotMotor extends SubsystemBase {

    private final TalonFX pivotMotor;
    private TalonFXConfiguration config;
    public DutyCycleOut controlRequest;
    public ArmFeedforward feedforward = new ArmFeedforward(Constants.Collector.PivotkS, Constants.Collector.PivotkG, Constants.Collector.PivotkV);
    public PivotMotor(){
        pivotMotor = new TalonFX(RobotMap.Collector.PIVOT_MOTOR_ID);
        setConfig();
    }

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

    public double getPivotVoltage() {
        return pivotMotor.getMotorVoltage().getValueAsDouble();
    }

    public void zeroEncoder() {
        pivotMotor.setPosition(0);
    }

    public void deployedEncoder() {
        pivotMotor.setPosition(Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS);
    }

    public boolean atSetPoint(double desiredSetpoint) {
        return Math.abs(getPosition() - desiredSetpoint) <= Constants.Collector.PIVOT_POSITION_TOLERANCE;
    }

    private void setConfig(){
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
        return pivotMotor.getDeviceTemp().getValueAsDouble() > Constants.MotorTemps.PIVOT_MOTOR_TEMP_WARNING_CELSIUS;
    }

    public void stopMotor() {
        pivotMotor.stopMotor();
    }

    public void holdPivotWithFF(){
       //stops motor and exits method if it's in deployed position
        if(getPosition() >= Constants.Collector.PIVOT_DEPLOYED_ENCODER_POS) {
            pivotMotor.stopMotor();
            return;
        }
        //Runs the following if the encoder value is not greater than or at deployed positon
            double angleRad = getPosition() * (2.0 * Math.PI);
            double FFvolts = feedforward.calculate(angleRad, 0);
            //converts volts into a percent
            double battery = RobotController.getBatteryVoltage();
            double percent = FFvolts / battery;
            //multiplies the percent by the current battery volt, gives a volt that the percent is using
            controlRequest = new DutyCycleOut(percent);
            //Uses the volt gotten from controlRequest to run the pivot motor
            pivotMotor.setControl(controlRequest);
    }

    private void sysIDVoltage(Voltage volt) {
        double volts = (volt.baseUnitMagnitude());
        double battery = RobotController.getBatteryVoltage();
        double percent = volts/battery;
        controlRequest = new DutyCycleOut(percent);
        pivotMotor.setControl(controlRequest);
    }

    private void sysIDLog(SysIdRoutineLog log) {
        MutVoltage voltMut = Volts.mutable(0);
        MutAngle posMut = Rotations.mutable(0);
        MutAngularVelocity vMut= RotationsPerSecond.mutable(0);

        log.motor("Pivot")
                .voltage(voltMut.mut_replace(getPivotVoltage(), Volts))
                .angularVelocity(vMut.mut_replace(pivotMotor.getVelocity().getValueAsDouble(), RotationsPerSecond))
                .angularPosition(posMut.mut_replace(pivotMotor.getPosition().getValueAsDouble(), Rotations));
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
