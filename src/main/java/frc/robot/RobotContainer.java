// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.arms.RetractArms;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.composite.FirstLevelScore;
import frc.robot.commands.composite.IntakeObject;
import frc.robot.commands.composite.PickupObject;
import frc.robot.commands.composite.SecondLevelScore;
import frc.robot.commands.composite.StowObjectIntake;
import frc.robot.commands.composite.ThirdLevelScore;
import frc.robot.commands.drivetrain.DriveToNearestGridPosition;
import frc.robot.commands.drivetrain.DriveToNearestSubstationAxis;
import frc.robot.commands.drivetrain.ManualDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;
import frc.robot.utils.DriverController;

/**
 * This class is where the bulk of the robot is declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here.
  private final Drivetrain drivetrain = new Drivetrain();
  private final LowerArm lowerArm = new LowerArm();
  private final UpperArm upperArm = new UpperArm();
  private final Intake intake = new Intake();
  private final Claw claw = new Claw();

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
   */
  private void configureButtonBindings() {
    this.drivetrain.setDefaultCommand(new ManualDrive(drivetrain, mainController));

    this.mainController.buttons.get(ControllerConstants.autoPickup).whileTrue(new ParallelCommandGroup(
        new PickupObject(lowerArm, upperArm, claw),
        new DriveToNearestSubstationAxis(drivetrain, mainController)));
    this.mainController.buttons.get(ControllerConstants.autoPickup).onFalse(new SequentialCommandGroup(
        new CloseClaw(claw),
        new RetractArms(lowerArm, upperArm)));

    this.mainController.buttons.get(ControllerConstants.intake)
        .whileTrue(new IntakeObject(lowerArm, upperArm, intake, claw));
    this.mainController.buttons.get(ControllerConstants.intake)
        .onFalse(new StowObjectIntake(lowerArm, upperArm, intake, claw));

    this.mainController.buttons.get(ControllerConstants.thirdScore)
        .whileTrue(new ThirdLevelScore(drivetrain, lowerArm, upperArm, claw));
    this.mainController.buttons.get(ControllerConstants.thirdScore)
        .onFalse(new RetractArms(lowerArm, upperArm));

    this.mainController.buttons.get(ControllerConstants.secondScore)
        .whileTrue(new SecondLevelScore(drivetrain, lowerArm, upperArm, claw));
    this.mainController.buttons.get(ControllerConstants.secondScore)
        .onFalse(new RetractArms(lowerArm, upperArm));

    this.mainController.buttons.get(ControllerConstants.firstScore)
        .whileTrue(new FirstLevelScore(drivetrain, lowerArm, upperArm, claw));
    this.mainController.buttons.get(ControllerConstants.firstScore)
        .onFalse(new RetractArms(lowerArm, upperArm));

    this.mainController.buttons.get(ControllerConstants.thirdAuto).whileTrue(new SequentialCommandGroup(
        new DriveToNearestGridPosition(drivetrain, mainController),
        new ThirdLevelScore(drivetrain, lowerArm, upperArm, claw)));
    this.mainController.buttons.get(ControllerConstants.thirdAuto)
        .onFalse(new RetractArms(lowerArm, upperArm));

    this.mainController.buttons.get(ControllerConstants.secondAuto).whileTrue(new SequentialCommandGroup(
        new DriveToNearestGridPosition(drivetrain, mainController),
        new SecondLevelScore(drivetrain, lowerArm, upperArm, claw)));
    this.mainController.buttons.get(ControllerConstants.secondAuto)
        .onFalse(new RetractArms(lowerArm, upperArm));

    this.mainController.buttons.get(ControllerConstants.firstAuto).whileTrue(new SequentialCommandGroup(
        new DriveToNearestGridPosition(drivetrain, mainController),
        new FirstLevelScore(drivetrain, lowerArm, upperArm, claw)));
    this.mainController.buttons.get(ControllerConstants.firstAuto)
        .onFalse(new RetractArms(lowerArm, upperArm));
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
