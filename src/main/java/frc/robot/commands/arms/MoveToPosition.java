// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveToPosition extends CommandBase {
  private Arm arm;
  private Supplier<Double> goalPositionSupplier;
  private boolean wait;

  /**
   * Creates a new MoveToPosition command. Works by setting the goal point of the
   * arm,
   * the arm then moves to that goal point.
   * 
   * @param arm      Arm to move
   * @param position Position to move to
   * @param wait     Whether or not to wait for the arm to arrive at the
   *                 goalpoint.
   */
  public MoveToPosition(Arm arm, double position, boolean wait) {
    this.arm = arm;
    this.goalPositionSupplier = () -> position;
    this.wait = wait;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.arm.setArmPosition(goalPositionSupplier.get());
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
    return !wait || arm.atSetpoint();
  }
}
