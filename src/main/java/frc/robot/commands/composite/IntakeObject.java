// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.arms.MoveArmsToState;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;

public class IntakeObject extends SequentialCommandGroup {

  /**
   * Creates a new command which intakes and grabs an object.
   * 
   * @param lowerArm
   * @param upperArm
   * @param intake
   * @param claw
   */
  public IntakeObject(LowerArm lowerArm, UpperArm upperArm, Intake intake, Claw claw) {
    addCommands(
        new ExtendIntake(intake),
        new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerIntake, ArmPositions.upperIntake),
        new OpenClaw(claw));
  }
}
