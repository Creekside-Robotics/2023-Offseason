// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;

/**
 * Creates a new SetClawMode object. This command sets the mode of the claw.
 */
public class SetClawMode extends InstantCommand {
  private Claw claw;
  private ClawMode clawMode;

  /**
   * Creates a new SetClawMode object. This command sets the mode of the claw.
   * 
   * @param claw     Claw whose mode will be set.
   * @param clawMode Mode to set the claw to.
   */
  public SetClawMode(Claw claw, ClawMode clawMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.clawMode = clawMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.claw.setClawMode(this.clawMode);
  }
}
