// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.arms.MoveArmsToState;
import frc.robot.commands.claw.WaitForObject;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
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
        new WaitForObject(claw),
        new WaitCommand(ClawConstants.openingDelay));
  }
}
