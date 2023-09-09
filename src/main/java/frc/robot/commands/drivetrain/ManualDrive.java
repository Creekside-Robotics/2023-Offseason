// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriverController;

public class ManualDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final DriverController controller;

  /**
   * Creates a new ManualDrive Command. This command sets drivetrain output to
   * that determined by the controller.
   * 
   * @param drivetrain Drivetrain to control.
   * @param controller Control to provide driver input.
   */
  public ManualDrive(Drivetrain drivetrain, DriverController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean intakeDisabled = !this.controller.getRawButton(ControllerConstants.intake);

    this.drivetrain.setDrivetrainOutput(
        this.controller.getDrivetrainOutput(intakeDisabled),
        intakeDisabled);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.setDrivetrainOutput(
        new ChassisSpeeds(),
        false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
