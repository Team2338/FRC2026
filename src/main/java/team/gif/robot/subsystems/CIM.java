// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class CIM extends SubsystemBase {
    public TalonSRX talon;
    /** Creates a new ExampleSubsystem. */
    public CIM() {
        talon = new TalonSRX(RobotMap.TALON_ID);
        talon.setInverted(true);
    }

    public void percent(double percent) {
        talon.set(TalonSRXControlMode.PercentOutput, percent);
    }

    public void stop() {
        talon.set(TalonSRXControlMode.PercentOutput, 0);
    }

}
