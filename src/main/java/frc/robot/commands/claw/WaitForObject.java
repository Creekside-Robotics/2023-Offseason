// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;

/** Creates a new WaitForObject. */
public class WaitForObject extends CommandBase {
  private Claw claw;

  /**
   * Creates a new WaitForObject Command. Command opens claw, then waits for an
   * object to be present before closing the claw.
   * 
   * @param claw Claw used in the command.
   */
  public WaitForObject(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.claw.setClawMode(ClawMode.sensorOpen);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.claw.hasObject();
  }
}
