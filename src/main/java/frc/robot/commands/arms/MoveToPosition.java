// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AbstractArm;

public class MoveToPosition extends CommandBase {
  /** Creates a new MoveToPosition. */
  private AbstractArm arm;
  private Supplier<Double> goalPositionSupplier;
  private boolean wait;

  public MoveToPosition(AbstractArm arm, double position, boolean wait) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.goalPositionSupplier = () -> position;
    this.wait = wait;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.arm.setArmPosition(goalPositionSupplier.get());
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !wait || arm.atSetpoint();
  }
}
