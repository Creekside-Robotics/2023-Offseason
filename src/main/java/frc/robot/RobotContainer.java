// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        this.drivetrain.setDefaultCommand(new ManualDrive(drivetrain, mainController));

        this.mainController.buttons.get(1).whileTrue(new ParallelCommandGroup(
                new PickupObject(lowerArm, upperArm, claw),
                //new DriveToNearestSubstationAxis(drivetrain, mainController)
                new WaitCommand(1000)
                ));
        this.mainController.buttons.get(1).onFalse(new SequentialCommandGroup(
          new CloseClaw(claw),
          new RetractArms(lowerArm, upperArm)));

        this.mainController.buttons.get(2).whileTrue(new IntakeObject(lowerArm, upperArm, intake, claw));
        this.mainController.buttons.get(2).onFalse(new StowObjectIntake(lowerArm, upperArm, intake, claw));

        this.mainController.buttons.get(7).whileTrue(new ThirdLevelScore(drivetrain, lowerArm, upperArm, claw));
        this.mainController.buttons.get(7).onFalse(new RetractArms(lowerArm, upperArm));

        this.mainController.buttons.get(9).whileTrue(new SecondLevelScore(drivetrain, lowerArm, upperArm, claw));
        this.mainController.buttons.get(9).onFalse(new RetractArms(lowerArm, upperArm));

        this.mainController.buttons.get(11).whileTrue(new FirstLevelScore(drivetrain, lowerArm, upperArm, claw));
        this.mainController.buttons.get(11).onFalse(new RetractArms(lowerArm, upperArm));

        this.mainController.buttons.get(8).whileTrue(new SequentialCommandGroup(
                new DriveToNearestGridPosition(drivetrain, mainController),
                new ThirdLevelScore(drivetrain, lowerArm, upperArm, claw)));
        this.mainController.buttons.get(8).onFalse(new RetractArms(lowerArm, upperArm));

        this.mainController.buttons.get(10).whileTrue(new SequentialCommandGroup(
                new DriveToNearestGridPosition(drivetrain, mainController),
                new SecondLevelScore(drivetrain, lowerArm, upperArm, claw)));
        this.mainController.buttons.get(10).onFalse(new RetractArms(lowerArm, upperArm));

        this.mainController.buttons.get(12).whileTrue(new SequentialCommandGroup(
                new DriveToNearestGridPosition(drivetrain, mainController),
                new FirstLevelScore(drivetrain, lowerArm, upperArm, claw)));
        this.mainController.buttons.get(12).onFalse(new RetractArms(lowerArm, upperArm));
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
