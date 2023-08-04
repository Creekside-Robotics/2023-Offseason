// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.arms.MoveArmsToState;
import frc.robot.commands.claw.CloseOnObject;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;

public class PickupObject extends SequentialCommandGroup {

  /**
   * Creates a new command which picks up an object from the shelf.
   * 
   * @param lowerArm
   * @param upperArm
   * @param claw
   */
  public PickupObject(LowerArm lowerArm, UpperArm upperArm, Claw claw) {
    addCommands(
        new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerPickup, ArmPositions.upperPickup),
        new OpenClaw(claw),
        new CloseOnObject(claw));
  }
}
