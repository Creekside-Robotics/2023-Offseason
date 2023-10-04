// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.arms.MoveArmsToState;
import frc.robot.commands.arms.RetractArms;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;

public class StowObjectIntake extends SequentialCommandGroup {

  /**
   * Creates a new command which stows the object collected by the intake.
   * 
   * @param lowerArm
   * @param upperArm
   * @param intake
   */
  public StowObjectIntake(LowerArm lowerArm, UpperArm upperArm, Intake intake, Claw claw) {
    addCommands(
        new OpenClaw(claw),
        new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerIntake, ArmPositions.upperIntake),
        new CloseClaw(claw),
        new RetractArms(lowerArm, upperArm));
  }
}
