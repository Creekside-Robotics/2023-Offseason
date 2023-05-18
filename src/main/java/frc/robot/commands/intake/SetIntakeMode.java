// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class SetIntakeMode extends InstantCommand {
  private Intake intake;
  private double intakeVelocity;
  private boolean intakeExtended;

  /**
   * Creates a new SetIntakeMode object, which sets the mode of the intake.
   * 
   * @param intake         Intake to set the mode of.
   * @param intakeVelocity Velocity to set the rollers of the intake to. Positive
   *                       is inward.
   * @param intakeExtended Whether or not the intake should be extended.
   */
  public SetIntakeMode(Intake intake, double intakeVelocity, boolean intakeExtended) {
    this.intake = intake;
    this.intakeVelocity = intakeVelocity;
    this.intakeExtended = intakeExtended;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intake.setIntakeMode(this.intakeVelocity, this.intakeExtended);
  }
}
