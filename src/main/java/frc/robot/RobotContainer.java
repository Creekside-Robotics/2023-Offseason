// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.DriveToPosePID;
import frc.robot.commands.drivetrain.ManualDrive;
import frc.robot.commands.drivetrain.SetDrivetrainOutput;
import frc.robot.commands.drivetrain.SetPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriverController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  private final DriverController mainController = new DriverController(Constants.DeviceIds.driverController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * This command is used to configure command and button bindings.
   * Button bindings are as follows:
   * 1: Set drivetrain output vector to <0.2, 0, 0>
   * 5: Drive robot to the axis of the far double substation
   * 6: Drive robot to the axis of the close double substation
   * 7: Reset robot pose
   * 8: Drive robot to the nearest scoring location.
   */
  private void configureButtonBindings() {
    this.drivetrain.setDefaultCommand(new ManualDrive(this.drivetrain, this.mainController));

    this.mainController.buttons.get(1).whileTrue(new SetDrivetrainOutput(
        this.drivetrain,
        new ChassisSpeeds(0.2, 0, 0),
        false));
    this.mainController.buttons.get(5).whileTrue(new DriveToPosePID(
        this.drivetrain,
        this.mainController,
        () -> Constants.fieldConstantsMap.get(DriverStation.getAlliance()).substationPositions[1],
        new boolean[] { true, false, true },
        new Pose2d(0, 0, new Rotation2d()),
        true));
    this.mainController.buttons.get(6).whileTrue(new DriveToPosePID(
        this.drivetrain,
        this.mainController,
        () -> Constants.fieldConstantsMap.get(DriverStation.getAlliance()).substationPositions[0],
        new boolean[] { true, false, true },
        new Pose2d(0, 0, new Rotation2d()),
        true));
    this.mainController.buttons.get(7).onTrue(new SetPose(this.drivetrain, new Pose2d()));
    this.mainController.buttons.get(8).whileTrue(new DriveToPosePID(
        this.drivetrain,
        this.mainController,
        () -> drivetrain.getClosestPose(Constants.fieldConstantsMap.get(DriverStation.getAlliance()).gridPositions),
        new boolean[] { true, true, true },
        new Pose2d(0.01, 0.01, new Rotation2d(0.01)),
        false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
