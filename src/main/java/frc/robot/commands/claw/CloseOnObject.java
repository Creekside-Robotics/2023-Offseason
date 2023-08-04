// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;

public class CloseOnObject extends CommandBase {
  /** Creates a new CloseOnObject. */
  private Claw claw;

  /**
   * Creates a new command that waits for an object to enter the claw, before
   * closing the claw and ending the command.
   * 
   * @param claw Claw object to use in the command.
   */
  public CloseOnObject(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.claw.setClawMode(ClawMode.open);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.claw.setClawMode(ClawMode.closed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.claw.getDistanceFromClaw() < ClawConstants.threshholdObjectDistance;
  }
}
