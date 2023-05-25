// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.Arm;

public class MoveArmsToState extends SequentialCommandGroup {

  /**
   * Creates a new MoveArmsToState command. The command moves the arms to the
   * desired state.
   * 
   * @param lower         Lower Arm
   * @param upper         Upper Arm
   * @param lowerPosition Desired end position for the lower arm.
   * @param upperPosition Desired end position for the upper arm.
   */
  public MoveArmsToState(Arm lower, Arm upper, double lowerPosition, double upperPosition) {
    addCommands(
        new MoveToPosition(upper, ArmPositions.upperTransitional, true),
        new MoveToPosition(lower, lowerPosition, true),
        new MoveToPosition(upper, upperPosition, true));
  }
}
