// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.arms.MoveArmsToState;
import frc.robot.commands.claw.SetClawMode;
import frc.robot.commands.claw.WaitForObject;
import frc.robot.commands.drivetrain.DriveToPosePID;
import frc.robot.commands.drivetrain.ManualDrive;
import frc.robot.commands.drivetrain.SetDrivetrainOutput;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;
import frc.robot.subsystems.Claw.ClawMode;
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
   * Button bindings are as follows:
   * 1: Set drivetrain output vector to <0.2, 0, 0>
   * 5: Drive robot to the axis of the far double substation
   * 6: Drive robot to the axis of the close double substation
   * 7: Reset robot pose
   * 8: Drive robot to the nearest scoring location.
   */
  private void configureButtonBindings() {
    Command manualDrive = new ManualDrive(drivetrain, mainController);
    Command moveToNearestScoringPosition = new DriveToPosePID(
        drivetrain,
        mainController,
        () -> drivetrain.getClosestPose(Constants.fieldConstantsMap.get(DriverStation.getAlliance()).gridPositions),
        new boolean[] { true, true, true },
        new Pose2d(0.01, 0.01, new Rotation2d(0.01)),
        false);
    Command moveToNearestPickupPosition = new DriveToPosePID(
        drivetrain,
        mainController,
        () -> drivetrain
            .getClosestPose(Constants.fieldConstantsMap.get(DriverStation.getAlliance()).substationPositions),
        new boolean[] { false, true, true },
        new Pose2d(0.01, 0.01, new Rotation2d(0.01)),
        true);

    Command firstLevelScore = new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerOne, ArmPositions.upperOne);
    Command secondLevelScore = new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerTwo, ArmPositions.upperTwo);
    Command thirdLevelScore = new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerThree, ArmPositions.upperThree);
    Command armsToPickup = new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerPickup, ArmPositions.upperPickup);
    Command armsToIntake = new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerIntake, ArmPositions.upperIntake);
    Command stowArms = new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerStowed, ArmPositions.upperStowed);

    Command extendIntake = new ExtendIntake(intake);
    Command retractIntake = new RetractIntake(intake);

    Command waitForObject = new WaitForObject(claw);
    Command openClaw = new SequentialCommandGroup(
        new SetClawMode(claw, ClawMode.open),
        new WaitCommand(ClawConstants.openingDelay));

    this.drivetrain.setDefaultCommand(manualDrive);

    this.mainController.buttons.get(1).whileTrue(new ParallelDeadlineGroup(
        waitForObject,
        armsToPickup,
        moveToNearestPickupPosition));
    this.mainController.buttons.get(1).onFalse(stowArms);

    this.mainController.buttons.get(2).whileTrue(new SequentialCommandGroup(
        extendIntake,
        armsToIntake,
        waitForObject));
    this.mainController.buttons.get(2).onFalse(new SequentialCommandGroup(
        stowArms,
        retractIntake));

    this.mainController.buttons.get(7).whileTrue(new SequentialCommandGroup(
        thirdLevelScore,
        openClaw));
    this.mainController.buttons.get(7).onFalse(stowArms);

    this.mainController.buttons.get(9).whileTrue(new SequentialCommandGroup(
        secondLevelScore,
        openClaw));
    this.mainController.buttons.get(9).onFalse(stowArms);

    this.mainController.buttons.get(11).whileTrue(new SequentialCommandGroup(
        firstLevelScore,
        openClaw));
    this.mainController.buttons.get(11).onFalse(stowArms);

    this.mainController.buttons.get(8).whileTrue(new SequentialCommandGroup(
        moveToNearestScoringPosition,
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(thirdLevelScore, openClaw),
            new SetDrivetrainOutput(drivetrain, new ChassisSpeeds(0.05, 0, 0), false))));
    this.mainController.buttons.get(8).onFalse(stowArms);

    this.mainController.buttons.get(10).whileTrue(new SequentialCommandGroup(
        moveToNearestScoringPosition,
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(secondLevelScore, openClaw),
            new SetDrivetrainOutput(drivetrain, new ChassisSpeeds(0.05, 0, 0), false))));
    this.mainController.buttons.get(10).onFalse(stowArms);

    this.mainController.buttons.get(12).whileTrue(new SequentialCommandGroup(
        moveToNearestScoringPosition,
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(firstLevelScore, openClaw),
            new SetDrivetrainOutput(drivetrain, new ChassisSpeeds(0.05, 0, 0), false))));
    this.mainController.buttons.get(12).onFalse(stowArms);
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
