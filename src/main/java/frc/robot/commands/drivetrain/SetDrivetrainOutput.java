// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetDrivetrainOutput extends CommandBase {
  private final Drivetrain drivetrain;
  private final ChassisSpeeds output;
  private final boolean fieldOriented;

  /**
   * Creates a new SetDrivetrain Output Command. The command sets the drivetrain
   * to the output specified for the duration of the command.
   * 
   * @param drivetrain    Drivetrain subsystem that will be controlled.
   * @param output        The ChassisSpeed object representing the motion vector
   *                      of the drivetrain.
   * @param fieldOriented Wether or not the motion vector is relative to the
   *                      field. If false, it will be
   *                      assumed that the vector is realtive to the robot.
   */
  public SetDrivetrainOutput(Drivetrain drivetrain, ChassisSpeeds output, boolean fieldOriented) {
    this.drivetrain = drivetrain;
    this.output = output;
    this.fieldOriented = fieldOriented;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.drivetrain.setDrivetrainOutput(this.output, this.fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.setDrivetrainOutput(new ChassisSpeeds(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
