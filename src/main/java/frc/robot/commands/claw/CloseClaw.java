// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;

public class CloseClaw extends SequentialCommandGroup {

  /**
   * Create a new command which closes the claw
   * 
   * @param claw
   */
  public CloseClaw(Claw claw) {
    addCommands(
        new SetClawMode(claw, ClawMode.closed),
        new WaitCommand(ClawConstants.openingDelay));
  }
}
