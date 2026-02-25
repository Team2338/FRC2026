// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Diagnostics extends SubsystemBase {

    public Diagnostics() {}

    public boolean swerveMotorTempHot() {
        return Robot.swerveDrive.fL.isDriveMotorHot() ||
                Robot.swerveDrive.fR.isDriveMotorHot() ||
                Robot.swerveDrive.rL.isDriveMotorHot() ||
                Robot.swerveDrive.rR.isDriveMotorHot();
    }

    public boolean mechanismMotorTempHot() {
        return Robot.collectMotor.isOverTemp() ||
                Robot.pivotMotor.isOverTemp() ||
                Robot.agitator.isOverTemp() ||
                Robot.indexer.isIndexOneOverTemp() ||
                Robot.indexer.isIndexTwoOverTemp() ||
                Robot.shooter.isLeftOverTemp() ||
                Robot.shooter.isMiddleOverTemp() ||
                Robot.shooter.isRightOverTemp();
    }

    public boolean anyMotorTempHot() {
        return swerveMotorTempHot() || mechanismMotorTempHot();
    }

}
