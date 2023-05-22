// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arms.RetractArms;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowObjectIntake extends SequentialCommandGroup {

  /**
   * Creates a new command which retracts the intake and stows the arms.
   * 
   * @param lowerArm
   * @param upperArm
   * @param intake
   */
  public StowObjectIntake(LowerArm lowerArm, UpperArm upperArm, Intake intake) {
    addCommands(
        new RetractArms(lowerArm, upperArm),
        new RetractIntake(intake));
  }
}