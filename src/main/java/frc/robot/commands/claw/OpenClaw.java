// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenClaw extends SequentialCommandGroup {

  /**
   * Creates a command which opens your claw with a predetermined delay.
   * 
   * @param claw Claw to open.
   */
  public OpenClaw(Claw claw) {
    addCommands(
        new SetClawMode(claw, ClawMode.open),
        new WaitCommand(ClawConstants.openingDelay));
  }
}
