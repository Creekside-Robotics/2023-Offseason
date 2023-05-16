// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class serves as the base for arm subsystems on the robot. */
public abstract class AbstractArm extends SubsystemBase {

    /**
     * Returns the current position of the arm.
     * @return Position of the arm in rotations from the zero-point.
     */
    public abstract double getArmPosition();

    /**
     * Sets the goal position of the arm, the arm will automatically move to this position.
     * @param position The position to set the arm to.
     */
    public abstract void setArmPosition(double position);

    /**
     * Tells whether or not the arm it at its setpoint.
     * @return Boolean representing whether or not the robot is at it's setpoint.
     */
    public abstract boolean atSetpoint();
}
