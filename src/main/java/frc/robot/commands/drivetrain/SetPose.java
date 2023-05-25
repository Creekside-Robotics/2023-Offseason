// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SetPose extends InstantCommand {
  private Drivetrain drivetrain;
  private Supplier<Pose2d> poseSupplier;

  /**
   * Creates a new SetPose command which sets the pose of the drivetrain.
   * 
   * @param drivetrain Drivetrain object.
   * @param pose       Pose to set the drivetrain to.
   */
  public SetPose(Drivetrain drivetrain, Pose2d pose) {
    this.drivetrain = drivetrain;
    this.poseSupplier = () -> pose;
  }

  /**
   * Creates a new SetPose command which sets the pose of the drivetrain.
   * 
   * @param drivetrain   Drivetrain object.
   * @param poseSupplier Suppier object, which provides a pose to set the
   *                     drivetrain to.
   */
  public SetPose(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.drivetrain.setDrivetrainPose(this.poseSupplier.get());
  }
}
