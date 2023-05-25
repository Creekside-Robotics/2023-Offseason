// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;

public class RetractArms extends MoveArmsToState {

    /**
     * Creates a new command which stows the arms
     * 
     * @param lower Lower arm.
     * @param upper Upper arm.
     */
    public RetractArms(LowerArm lower, UpperArm upper) {
        super(lower, upper, ArmPositions.lowerStowed, ArmPositions.upperStowed);
    }
}
