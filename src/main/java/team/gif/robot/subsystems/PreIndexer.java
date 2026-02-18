// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class PreIndexer extends SubsystemBase {

    private SparkFlex preIndexerMotor;

    public PreIndexer() {
        preIndexerMotor = new SparkFlex(RobotMap.Shooter.PRE_INDEXER, SparkLowLevel.MotorType.kBrushless);
    }

    public void runPercent(double percent) {
        preIndexerMotor.set(percent);
    }

    public void stopMotor(){
        preIndexerMotor.stopMotor();
    }

}
