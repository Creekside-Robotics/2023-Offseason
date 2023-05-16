// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class serves as the base for arm subsystems on the robot. */
public abstract class AbstractArm extends SubsystemBase {
    abstract double getArmPosition();
    abstract void setArmPosition();
}
